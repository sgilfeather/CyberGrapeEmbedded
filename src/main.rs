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

use alloc::format;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp_pico::entry;
use rp_pico::hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use defmt::*;
use defmt_rtt as _;
use nb::Error;

// Some traits we need
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

extern crate alloc;
use cg::HardwareSpinlock;
use talc::*;

use alloc::vec::Vec;

static mut ARENA: [u8; 16384] = [0; 16384];

#[global_allocator]
static ALLOCATOR: Talck<HardwareSpinlock<13>, ClaimOnOom> =
    Talc::new(unsafe { ClaimOnOom::new(Span::from_const_array(core::ptr::addr_of!(ARENA))) })
        .lock();

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
#[entry]
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
        UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    let (uart1, writer) = hal::uart::UartPeripheral::new(
        pac.UART1,
        (pins.gpio8.into_function(), pins.gpio9.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap()
    .split();

    let mut raw_buf0: [u8; 512] = [0; 512];
    let mut raw_buf1: [u8; 512] = [0; 512];
    let mut read_buf0: Vec<u8> = Vec::new();
    let mut read_buf1: Vec<u8> = Vec::new();

    loop {
        let read0 = uart0
            .read_raw(&mut raw_buf0)
            .unwrap_or_else(|err| match err {
                Error::WouldBlock => 0,
                Error::Other(err) => {
                    error!("FAIL {}", format!("{:?}", err).as_str());
                    core::panic!();
                }
            });
        for i in 0..read0 {
            match raw_buf0[i] {
                b'\n' => {
                    read_buf0.push(b'\n');
                    let rest = writer.write_raw(&read_buf0).unwrap();
                    read_buf0.clear();
                    // read_buf0.extend(rest);
                }
                c => {
                    read_buf0.push(c);
                }
            }
        }

        let read1 = uart1
            .read_raw(&mut raw_buf1)
            .unwrap_or_else(|err| match err {
                Error::WouldBlock => 0,
                Error::Other(err) => {
                    error!("FAIL {}", format!("{:?}", err).as_str());
                    core::panic!();
                }
            });
        for i in 0..read1 {
            match raw_buf1[i] {
                b'\n' => {
                    read_buf1.push(b'\n');
                    writer.write_raw(&read_buf1);
                    read_buf1.clear();
                }
                c => {
                    read_buf1.push(c);
                }
            }
        }
    }
}
