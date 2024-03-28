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
use menu::*;
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
use embedded_hal::{adc::OneShot, digital::v2::OutputPin};
use heapless::String;

// CLI Root Menu Struct Initialization
const ROOT_MENU: Menu<SerialPort<'_, dyn UsbBus>> = Menu {
    label: "root",
    items: &[&Item {
        item_type: ItemType::Callback {
            function: hello_name,
            parameters: &[Parameter::Mandatory {
                parameter_name: "name",
                help: Some("Enter your name"),
            }],
        },
        command: "hw",
        help: Some("This is an embedded CLI terminal. Check the summary for the list of supported commands"),
    }],
    entry: None,
    exit: None,
};
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

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let mut temperature_sensor = adc.take_temp_sensor().unwrap();

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

    let mut led_pin = pins.gpio28.into_push_pull_output();

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

    // Create a buffer to store CLI input
    let mut clibuf = [0u8; 64];
    // Instantiate CLI runner with root menu, buffer, and uart
    let mut r = Runner::new(ROOT_MENU, &mut clibuf, &serial);

    loop {
        //in case the usb channel is closed return to the previous state

        // A welcome message at the beginning
        let interval = Instant::from_ticks(4_000_000);

        // Check for new data

        if usb_dev.poll(&mut [&mut serial]) {
            // Create single element buffer for UART characters
            let mut buf = [0_u8; 1];
            // Read single byte from UART
            if serial.read(&mut buf).unwrap() != 0 {
                // Pass read byte to CLI runner for processing
                r.input_byte(buf[0]);
            }
        }
    }
}

// Callback function for hw commans
fn hello_name<'a>(
    _menu: &Menu<SerialPort<'_, dyn UsbBus>>,
    item: &Item<SerialPort<'_, dyn UsbBus>>,
    args: &[&str],
    context: &mut SerialPort<'_, dyn UsbBus>,
) {
    // Print to console passed "name" argument
    writeln!(
        context,
        "Hello, {}!",
        argument_finder(item, args, "name").unwrap().unwrap()
    )
    .unwrap();
}
// End of file
