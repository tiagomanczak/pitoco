//! # PITOCO
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device which receives commands
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use hal::rtc::{DayOfWeek, RealTimeClock};
use rp2040_hal::rtc::DateTime;
// The macro for our start-up function
use rp_pico::{
    entry,
    hal::{adc::Adc, Sio},
};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use rp2040_hal::timer::Instant;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::fmt::Write;

use heapless::String;

mod pitoco;
use pitoco::Pitoco;
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised..
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = Sio::new(pac.SIO);

    let adc = Adc::new(pac.ADC, &mut pac.RESETS);

    //let mut temperature_sensor = adc.take_temp_sensor().unwrap();

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let initial_date: DateTime = DateTime {
        year: 1,
        month: 1,
        day: 1,
        day_of_week: DayOfWeek::Monday,
        hour: 0,
        minute: 0,
        second: 0,
    };

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let rtc = RealTimeClock::new(pac.RTC, clocks.rtc_clock, &mut pac.RESETS, initial_date).unwrap();

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led_pin = pins.gpio28.into_push_pull_output();

    let mut cmd_rcv_buf: [u8; 64] = [0; 64];
    let mut text: String<128> = String::new();
    let mut pitoco = Pitoco::new(&mut cmd_rcv_buf, rtc, led_pin, adc);

    #[cfg(feature = "rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID using 0x0424 and 0x274e so the linux driver disables the echo
    //reference: https://michael.stapelberg.ch/posts/2021-04-27-linux-usb-virtual-serial-cdc-acm/
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x0424, 0x274e))
        .manufacturer("Augsburg Rust Meetup")
        .product("Pitoco Usb Serial")
        .serial_number("Pitoco01")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut greetins_done = false;

    loop {
        //in case the usb channel is closed return to the previous state

        // A welcome message at the beginning
        let interval = Instant::from_ticks(4_000_000);

        if timer.get_counter() > interval && !greetins_done {
            let _ = serial.write(b"Hello, World!\n");
            greetins_done = true;
            let time = timer.get_counter();
            let mut text: String<64> = String::new();
            let _ = writeln!(&mut text, "Current timer ticks: {}", time);

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }

        // Check for new data

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    //echo the data back so the user see what it is typing in the terminal
                    let _ = serial.write(&buf[..count]);

                    //to debug we see the number of bytes and the char each loop
                    #[cfg(feature = "debug")]
                    {
                        writeln!(&mut text, "Count: {} Char: {}", count, buf[0]).unwrap();
                        let _ = serial.write(text.as_bytes());
                    }

                    pitoco.process(buf[0], &mut text).unwrap();
                    //in case od debug we see the content of the pitoco buffer
                    #[cfg(feature = "debug")]
                    {
                        let mut wr_ptr = &pitoco.buffer[..pitoco.index];
                        while !wr_ptr.is_empty() {
                            match serial.write(wr_ptr) {
                                Ok(len) => wr_ptr = &wr_ptr[len..],
                                // On error, just drop unwritten data.
                                // One possible error is Err(WouldBlock), meaning the USB
                                // write buffer is full.
                                Err(_) => break,
                            };
                        }
                    }
                }
            }
        }
    }
}

// End of file
