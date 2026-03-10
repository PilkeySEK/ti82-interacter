use serialport::SerialPort;
use std::io;
use std::time::Duration;

// ── Machine & Command IDs ────────────────────────────────────────────────────
const MID_PC: u8 = 0x02;
#[expect(unused)]
const MID_TI: u8 = 0x82;

const CID_VAR: u8 = 0x06; // Variable header
const CID_CTS: u8 = 0x09; // Continue / ready to receive
const CID_XDP: u8 = 0x15; // Data packet
const CID_ACK: u8 = 0x56; // Acknowledgement
const CID_ERR: u8 = 0x5A; // Checksum error – resend
const CID_EOT: u8 = 0x92; // End of transmission

// ── Error type ───────────────────────────────────────────────────────────────
#[derive(Debug)]
pub enum Ti82Error {
    Io(io::Error),
    InvalidFile(&'static str),
    UnexpectedResponse { expected: u8, got: u8 },
    CalculatorError(u8),
    ChecksumMismatch,
    TooManyRetries,
}

impl From<io::Error> for Ti82Error {
    fn from(e: io::Error) -> Self {
        Ti82Error::Io(e)
    }
}

impl std::fmt::Display for Ti82Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ti82Error::Io(e) => write!(f, "I/O error: {e}"),
            Ti82Error::InvalidFile(s) => write!(f, "Invalid .82g file: {s}"),
            Ti82Error::UnexpectedResponse { expected, got } => write!(
                f,
                "Unexpected response: expected CID {expected:#04x}, got {got:#04x}"
            ),
            Ti82Error::CalculatorError(code) => {
                write!(f, "Calculator rejected transfer (code {code:#04x})")
            }
            Ti82Error::ChecksumMismatch => write!(f, "Checksum mismatch"),
            Ti82Error::TooManyRetries => write!(f, "Too many retransmission attempts"),
        }
    }
}

// ── Parsed variable entry from the .82g file ────────────────────────────────
#[derive(Debug)]
struct Ti82Var {
    var_type: u8,
    name: [u8; 8], // null-padded to 8 bytes
    data: Vec<u8>,
}

// ── Checksum: lower 16 bits of the sum of all data bytes ────────────────────
fn checksum(data: &[u8]) -> u16 {
    data.iter().map(|&b| b as u32).sum::<u32>() as u16
}

// ── Read exactly n bytes from the port ──────────────────────────────────────
fn read_exact_port(port: &mut dyn SerialPort, buf: &mut [u8]) -> Result<(), Ti82Error> {
    let mut read = 0;
    while read < buf.len() {
        match port.read(&mut buf[read..]) {
            Ok(0) => {} // timeout – retry
            Ok(n) => read += n,
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(Ti82Error::Io(e)),
        }
    }
    Ok(())
}

// ── Receive a 4-byte header packet from the TI ──────────────────────────────
// Returns (mid, cid, param_lo, param_hi)
fn recv_packet_header(port: &mut dyn SerialPort) -> Result<(u8, u8, u8, u8), Ti82Error> {
    let mut buf = [0u8; 4];
    read_exact_port(port, &mut buf)?;
    Ok((buf[0], buf[1], buf[2], buf[3]))
}

// ── Send a packet with no data (just the 4-byte header) ─────────────────────
fn send_simple(port: &mut dyn SerialPort, cid: u8, p_lo: u8, p_hi: u8) -> Result<(), Ti82Error> {
    port.write_all(&[MID_PC, cid, p_lo, p_hi])?;
    Ok(())
}

// ── Expect an ACK from the TI (CID=56) ──────────────────────────────────────
fn expect_ack(port: &mut dyn SerialPort) -> Result<(), Ti82Error> {
    let (_, cid, _, _) = recv_packet_header(port)?;
    if cid != CID_ACK {
        return Err(Ti82Error::UnexpectedResponse {
            expected: CID_ACK,
            got: cid,
        });
    }
    Ok(())
}

// ── Build and send a VAR header packet ──────────────────────────────────────
// Packet data = [ data_len_lo, data_len_hi, var_type, name(8 bytes) ]  (11 bytes)
fn send_var_header(port: &mut dyn SerialPort, var: &Ti82Var) -> Result<(), Ti82Error> {
    let data_len = var.data.len() as u16;
    let mut payload = Vec::with_capacity(11);
    payload.push((data_len & 0xFF) as u8);
    payload.push((data_len >> 8) as u8);
    payload.push(var.var_type);
    payload.extend_from_slice(&var.name);

    let chk = checksum(&payload);
    // Packet: MID CID PL_LO PL_HI <payload> CHK_LO CHK_HI
    // PL = 0x0B (11) – the fixed var-header data length
    let mut pkt = vec![MID_PC, CID_VAR, 0x0B, 0x00];
    pkt.extend_from_slice(&payload);
    pkt.push((chk & 0xFF) as u8);
    pkt.push((chk >> 8) as u8);

    port.write_all(&pkt)?;
    Ok(())
}

// ── Send an XDP (data) packet, retrying on ERR ──────────────────────────────
fn send_xdp(port: &mut dyn SerialPort, data: &[u8]) -> Result<(), Ti82Error> {
    let len = data.len() as u16;
    let chk = checksum(data);

    let mut pkt = vec![MID_PC, CID_XDP, (len & 0xFF) as u8, (len >> 8) as u8];
    pkt.extend_from_slice(data);
    pkt.push((chk & 0xFF) as u8);
    pkt.push((chk >> 8) as u8);

    const MAX_RETRIES: u8 = 3;
    for attempt in 0..MAX_RETRIES {
        if attempt > 0 {
            // Re-send the same packet
            port.write_all(&pkt)?;
        } else {
            port.write_all(&pkt)?;
        }

        let (_, cid, _, _) = recv_packet_header(port)?;
        match cid {
            CID_ACK => return Ok(()),
            CID_ERR => continue, // TI asked us to resend
            other => {
                return Err(Ti82Error::UnexpectedResponse {
                    expected: CID_ACK,
                    got: other,
                });
            }
        }
    }
    Err(Ti82Error::TooManyRetries)
}

// ── Parse the .82g file into a list of Ti82Var ──────────────────────────────
fn parse_82g(bytes: &[u8]) -> Result<Vec<Ti82Var>, Ti82Error> {
    // Minimum size: 55-byte header + 2-byte checksum
    if bytes.len() < 57 {
        return Err(Ti82Error::InvalidFile("file too short"));
    }
    // Verify signature
    if &bytes[0..8] != b"**TI82**" {
        return Err(Ti82Error::InvalidFile("bad signature"));
    }
    // Further signature: {1Ah, 0Ah, 00h}
    if bytes[8..11] != [0x1A, 0x0A, 0x00] {
        return Err(Ti82Error::InvalidFile("bad secondary signature"));
    }

    // Data section length (offset 53, 2 bytes LE)
    let data_section_len = u16::from_le_bytes([bytes[53], bytes[54]]) as usize;

    // Verify file size: header(55) + data_section + checksum(2) = 57 + data_section_len
    if bytes.len() < 55 + data_section_len + 2 {
        return Err(Ti82Error::InvalidFile("file truncated"));
    }

    // Verify file checksum
    let data_section = &bytes[55..55 + data_section_len];
    let file_chk = checksum(data_section);
    let stored_chk = u16::from_le_bytes([
        bytes[55 + data_section_len],
        bytes[55 + data_section_len + 1],
    ]);
    if file_chk != stored_chk {
        return Err(Ti82Error::ChecksumMismatch);
    }

    // Parse variable entries from the data section
    let mut vars = Vec::new();
    let mut pos = 0usize;

    while pos + 15 <= data_section.len() {
        // Offset 0–1: always 0x000B (11)
        let _header_len = u16::from_le_bytes([data_section[pos], data_section[pos + 1]]);

        // Offset 2–3: variable data length
        let var_data_len =
            u16::from_le_bytes([data_section[pos + 2], data_section[pos + 3]]) as usize;

        // Offset 4: variable type
        let var_type = data_section[pos + 4];

        // Offset 5–12: variable name (8 bytes, null-padded)
        let mut name = [0u8; 8];
        name.copy_from_slice(&data_section[pos + 5..pos + 13]);

        // Offset 13–14: duplicate data length (we skip / ignore)
        // Offset 15..15+var_data_len: variable data
        let data_start = pos + 15;
        let data_end = data_start + var_data_len;
        if data_end > data_section.len() {
            return Err(Ti82Error::InvalidFile("variable data extends beyond file"));
        }

        vars.push(Ti82Var {
            var_type,
            name,
            data: data_section[data_start..data_end].to_vec(),
        });

        pos = data_end;
    }

    if vars.is_empty() {
        return Err(Ti82Error::InvalidFile("no variables found in file"));
    }

    Ok(vars)
}

// ── Send one variable following protocol section 3.9 ────────────────────────
fn send_one_var(port: &mut dyn SerialPort, var: &Ti82Var) -> Result<(), Ti82Error> {
    // Step 1 – PC sends VAR header
    send_var_header(port, var)?;

    // Step 2 – TI ACKs
    expect_ack(port)?;

    // Step 3 – TI sends CTS (wait for user to accept on calc)
    let (_, cid, _, _) = recv_packet_header(port)?;
    if cid != CID_CTS {
        return Err(Ti82Error::UnexpectedResponse {
            expected: CID_CTS,
            got: cid,
        });
    }

    // Step 4 – PC ACKs
    send_simple(port, CID_ACK, 0x00, 0x00)?;

    // Step 5 – PC sends XDP (data); TI ACKs (handled inside send_xdp)
    send_xdp(port, &var.data)?;

    Ok(())
}

// ── Public entry point ───────────────────────────────────────────────────────
/// Send all variables contained in a `.82g` file to the TI-82 connected to
/// `port`. The port should already be opened and configured (9600 baud, 8N1).
///
/// # Example
/// ```no_run
/// let port = serialport::new("/dev/ttyUSB0", 9600)
///     .timeout(Duration::from_secs(30))
///     .open()
///     .expect("Failed to open port");
///
/// let file_bytes = std::fs::read("program.82g").expect("Failed to read file");
/// send_ti_program(port, &file_bytes).expect("Transfer failed");
/// ```
pub fn send_ti_program(mut port: Box<dyn SerialPort>, file_bytes: &[u8]) -> Result<(), Ti82Error> {
    let vars = parse_82g(file_bytes)?;
    let count = vars.len();

    for (i, var) in vars.iter().enumerate() {
        send_one_var(&mut *port, var)?;

        // After each variable (except the last) the TI may send ACK for the
        // data packet – already consumed in send_xdp. No extra step needed.
        let _ = i; // suppress unused warning
    }

    // Step 7 – PC sends EOT after all variables
    send_simple(&mut *port, CID_EOT, 0x00, 0x00)?;

    // Step 8 – TI ACKs EOT
    expect_ack(&mut *port)?;

    println!("Transfer complete: {count} variable(s) sent.");
    Ok(())
}

// ── Usage example ────────────────────────────────────────────────────────────
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <serial_port> <file.82g>", args[0]);
        eprintln!("  e.g. {} /dev/ttyUSB0 program.82g", args[0]);
        std::process::exit(1);
    }

    let port_name = &args[1];
    let file_path = &args[2];

    let port = serialport::new(port_name, 9600)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .timeout(Duration::from_secs(30))
        .open()?;

    let file_bytes = std::fs::read(file_path)?;
    send_ti_program(port, &file_bytes).unwrap();

    Ok(())
}
