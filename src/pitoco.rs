use core::fmt::Write;
use heapless::String;
use rp2040_hal::{
    adc::TempSense,
    gpio::{bank0::Gpio28, FunctionSio, Pin, PullDown, SioOutput},
    rtc::RealTimeClock,
    Adc,
};

use embedded_hal::{adc::OneShot, digital::v2::OutputPin};

pub struct Pitoco<'a> {
    buffer: &'a mut [u8; 64],
    index: usize,
    rtc: RealTimeClock,
    led_pin: Pin<Gpio28, FunctionSio<SioOutput>, PullDown>,
    adc: Adc,
    temperature_sensor: Option<TempSense>,
}
pub enum PitocoCommand {
    LedOn,
    LedOff,
    Temperature,
    DateTime,
    Unknown,
}

impl<'a> Pitoco<'a> {
    pub fn new(
        buffer: &'a mut [u8; 64],
        rtc: RealTimeClock,
        led_pin: Pin<Gpio28, FunctionSio<SioOutput>, PullDown>,
        adc_in: Adc,
    ) -> Pitoco {
        Pitoco {
            buffer: buffer,
            index: 0,
            rtc: rtc,
            led_pin: led_pin,
            adc: adc_in,
            temperature_sensor: None,
        }
    }

    fn rcv(&mut self, byte: u8) -> Option<PitocoCommand> {
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

    pub fn process(
        &mut self,
        byte: u8,
        mut text: &mut String<128>,
    ) -> Result<(), core::fmt::Error> {
        if let Some(cmd) = self.rcv(byte) {
            text.clear();
            match cmd {
                PitocoCommand::LedOn => {
                    writeln!(text, "\nLedOn")?;
                    self.led_pin.set_high().unwrap();
                }
                PitocoCommand::LedOff => {
                    writeln!(text, "\nLedOff")?;
                    self.led_pin.set_low().unwrap();
                }
                PitocoCommand::Temperature => {
                    if let None = self.temperature_sensor {
                        self.temperature_sensor = Some(self.adc.take_temp_sensor().unwrap());
                    }
                    let temp: u16 = self
                        .adc
                        .read(self.temperature_sensor.as_mut().unwrap())
                        .unwrap();

                    writeln!(&mut text, "\nTemp: {}", temp)?;
                }
                PitocoCommand::DateTime => {
                    let now = self.rtc.now().unwrap();
                    writeln!(text, "\n\rDateTime")?;
                    writeln!(
                        &mut text,
                        "\n\rTime: {}:{}:{}",
                        now.hour, now.minute, now.second
                    )?;
                    writeln!(
                        &mut text,
                        "\n\rDate: {}-{}-{}",
                        now.year, now.month, now.day
                    )?;
                }
                PitocoCommand::Unknown => {
                    writeln!(&mut text, "Unknown Command")?;
                }
            }
            writeln!(text, "\n\rpitoco>>")?;
            return Ok(());
        }
        return Ok(());
    }
}
