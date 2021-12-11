#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use heapless::String;
use panic_halt as _;
use rp_bongo::hal::{self, pac, prelude::*};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static FERRIS1: &[u8] = include_bytes!("../ferris1.raw");
static FERRIS2: &[u8] = include_bytes!("../ferris2.raw");

#[entry]
fn main() -> ! {
    // Device initialization
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = hal::sio::Sio::new(pac.SIO);

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_bongo::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp_bongo::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup the screen
    let screen_sda = pins.screen_sda.into_mode::<hal::gpio::FunctionI2C>();
    let screen_scl = pins.screen_scl.into_mode::<hal::gpio::FunctionI2C>();
    let screen_i2c = hal::I2C::i2c1(
        pac.I2C1,
        screen_sda,
        screen_scl,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    let interface = I2CDisplayInterface::new(screen_i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Load ferris
    let ferris1: ImageRawLE<BinaryColor> = ImageRawLE::new(FERRIS1, 128);
    let ferris1_img = Image::new(&ferris1, Point::zero());

    let ferris2: ImageRawLE<BinaryColor> = ImageRawLE::new(FERRIS2, 128);
    let ferris2_img = Image::new(&ferris2, Point::zero());

    let ferris_imgs = [ferris1_img, ferris2_img];

    // let mut led = pins.led.into_push_pull_output();

    // Start ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut audio_in = pins.audio_in.into_floating_input();
    let mut is_high = false;

    // Draw ferris
    let mut ferris_cur = 0;
    ferris_imgs[ferris_cur].draw(&mut display).unwrap();
    display.flush().unwrap();

    let mut counter = 0;

    loop {
        let audio_val: u16 = adc.read(&mut audio_in).unwrap();

        if !is_high && audio_val > 0x0150 {
            is_high = true;
            // Invert the ferris
            ferris_cur ^= 1;
            display.flush().unwrap();
        } else if is_high && audio_val < 0x0150 {
            is_high = false;
        }

        counter += 1;

        if counter == 20 {
            counter = 0;
            let mut text: String<4> = String::from("");
            let _ = write!(text, "{:04X}", audio_val);

            display.clear();
            ferris_imgs[ferris_cur].draw(&mut display).unwrap();
            Text::with_baseline(text.as_str(), Point::new(64, 0), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            display.flush().unwrap();
        }

        delay.delay_ms(5);
    }
}
