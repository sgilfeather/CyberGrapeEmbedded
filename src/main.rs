//! # UART Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp_pico::hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use defmt::*;
use defmt_rtt as _;

// Some traits we need
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};
use rp_pico::hal::uart::ReadErrorType;

use core::str;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .map_err(|_| "could not get it to work")
    .unwrap();

    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // hal::uart::UartPeripheral::new(pac.)

    let uart0 = hal::uart::UartPeripheral::new(
        pac.UART0,
        (pins.gpio12.into_function(), pins.gpio13.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        UartConfig::new(19200.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    let uart1 = hal::uart::UartPeripheral::new(
        pac.UART1,
        (pins.gpio8.into_function(), pins.gpio9.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        UartConfig::new(19200.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    let mut raw_buf0: [u8; 512] = [0; 512];
    let mut raw_buf1: [u8; 512] = [0; 512];
    let mut read_buf0: [u8; 512] = [0; 512];
    let mut read_buf1: [u8; 512] = [0; 512];
    let mut pos0: usize = 0;
    let mut pos1: usize = 0;

    loop {
        uart0
            .read_full_blocking(&mut raw_buf0)
            .map_err(|e| {
                let e_str = match e {
                    ReadErrorType::Break => "break",
                    ReadErrorType::Framing => "framing",
                    ReadErrorType::Overrun => "overrun",
                    ReadErrorType::Parity => "parity",
                };
                error!("uart0 {}", e_str);
                None::<&str>
            })
            .expect("should not fail");

        uart1
            .read_full_blocking(&mut raw_buf1)
            .map_err(|e| {
                let e_str = match e {
                    ReadErrorType::Break => "break",
                    ReadErrorType::Framing => "framing",
                    ReadErrorType::Overrun => "overrun",
                    ReadErrorType::Parity => "parity",
                };
                error!("uart1 {}", e_str);
                None::<&str>
            })
            .expect("should not fail");

        match raw_buf0[0] {
            b'\n' => {
                println!("{}", str::from_utf8(&read_buf0).unwrap());
                pos0 = 0;
            }
            c => {
                read_buf0[pos0] = c;
                pos0 += 1;
            }
        }

        match raw_buf1[0] {
            b'\n' => {
                println!("{}", str::from_utf8(&read_buf1).unwrap());
                pos1 = 0;
            }
            c => {
                read_buf1[pos1] = c;
                pos1 += 1;
            }
        }

        println!("buf0: {}", str::from_utf8(&raw_buf0).unwrap());
        println!("buf1: {}", str::from_utf8(&raw_buf1).unwrap());
    }
}

// End of file
