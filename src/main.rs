//! TODO

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

#[allow(unused)]
use defmt::*;
use defmt_rtt as _;
use nb::Error;

// Some traits we need
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

// All of the things we need to get the heap allocator working
use cg::HardwareSpinlock;
use rp_pico::hal::uart::{ReadError, Reader, UartDevice, ValidUartPinout, Writer};
use talc::{ClaimOnOom, Span, Talc, Talck};

extern crate alloc;
use alloc::vec::Vec;

// Some stack space to store our heap
static mut ARENA: [u8; 16384] = [0; 16384];

// Instantiate an allocator, give it the arena, and tell Rust that we want this
// to be our global allocator for all dynamically-sized types.
// Those dynamically sized types need to come from alloc, not std.
#[global_allocator]
static ALLOCATOR: Talck<HardwareSpinlock<13>, ClaimOnOom> =
    Talc::new(unsafe { ClaimOnOom::new(Span::from_const_array(core::ptr::addr_of!(ARENA))) })
        .lock();

/// Entry point to our bare-metal application.
///
/// The `#[rp_pico::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp_pico::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .expect("Should be able to initialize clocks");

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

    // Instantiate our two UART channels, then split them so that we have a
    // two readers and a writer (we throw away the uart0 writer)
    let (uart0, _) = hal::uart::UartPeripheral::new(
        pac.UART0,
        (pins.gpio12.into_function(), pins.gpio13.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap()
    .split();

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

    // These bad boys are going to hold the raw data we get out of the UART
    // channel
    let mut raw_buf0: [u8; 512] = [0; 512];
    let mut raw_buf1: [u8; 512] = [0; 512];

    // Then these ones will be where we assemble the messages
    let mut read_buf0: Vec<u8> = Vec::new();
    let mut read_buf1: Vec<u8> = Vec::new();

    loop {
        let read0 = perform_read(&mut raw_buf0, &uart0).expect("Read from UART0 should succeed");
        handle_read(read0, &raw_buf0, &mut read_buf0, &writer);

        let read1 = perform_read(&mut raw_buf1, &uart1).expect("Read from UART1 should succeed");
        handle_read(read1, &raw_buf1, &mut read_buf1, &writer);
    }
}

/// Attempts to read from the UART Reader, but instead of having a special error
/// case when the read queue is empty, simply returns 0 to indicate that no
/// characters have been read. Any other read errors are propigated.
fn perform_read<'a, D, P>(
    raw_buf: &'a mut [u8],
    reader: &Reader<D, P>,
) -> Result<usize, ReadError<'a>>
where
    D: UartDevice,
    P: ValidUartPinout<D>,
{
    match reader.read_raw(raw_buf) {
        Ok(len) => Ok(len),
        Err(Error::WouldBlock) => Ok(0),
        Err(Error::Other(other)) => Err(other),
    }
}

/// Handles processing a read from the UART, appending to the read buffer and
/// flushing that buffer out to the writer if it sees a newline character
fn handle_read<D, P>(read_len: usize, raw_buf: &[u8], read_buf: &mut Vec<u8>, writer: &Writer<D, P>)
where
    D: UartDevice,
    P: ValidUartPinout<D>,
{
    for &c in raw_buf.iter().take(read_len) {
        read_buf.push(c);

        if c == b'\n' {
            let _rest = writer
                .write_raw(read_buf)
                .expect("Write to UART1 should succeed");
            read_buf.clear();
        }
    }
}
