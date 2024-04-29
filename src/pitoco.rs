pub const PREFIX: &[u8] = b"\n\rpitoco>>";

pub struct Pitoco<'a> {
    buffer: &'a mut [u8; 64],
    index: usize,
}
pub enum PitocoCommand {
    LedOn,
    LedOff,
    Temperature,
    DateTime,
    Unknown,
}

impl<'a> Pitoco<'a> {
    pub fn new(buffer: &'a mut [u8; 64]) -> Pitoco {
        Pitoco {
            buffer: buffer,
            index: 0,
        }
    }

    pub fn rcv(&mut self, byte: u8) -> Option<PitocoCommand> {
        if byte == 13 {
            return self.parse_cmd();
        }

        self.rcv_u8(byte);

        None
    }

    fn rcv_u8(&mut self, byte: u8) {
        if let Some(buf_byte) = self.buffer.get_mut(self.index) {
            *buf_byte = byte;
        } else {
            panic!("Overload");
        }
        self.index += 1;
    }

    fn parse_cmd(&mut self) -> Option<PitocoCommand> {
        let mut return_comand: Option<PitocoCommand> = None;

        if let Some(str_cmd) = core::str::from_utf8(&self.buffer[..self.index]).ok() {
            return_comand = match str_cmd {
                "led on" => Some(PitocoCommand::LedOn),
                "led off" => Some(PitocoCommand::LedOff),
                "temp" => Some(PitocoCommand::Temperature),
                "datetime" => Some(PitocoCommand::DateTime),
                _ => Some(PitocoCommand::Unknown),
            }
        }
        self.index = 0;
        return_comand
    }
}
