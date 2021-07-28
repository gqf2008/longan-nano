#![no_std]
#![no_main]

use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    pixelcolor::PixelColor,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
    text::{Baseline, Text, TextStyleBuilder},
};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use epd_waveshare::{
    epd2in7b_v2::{Display2in7b, Epd2in7b},
    prelude::*,
};
use gd32vf103xx_hal::pac::SPI1;
use gd32vf103xx_hal::rcu::Rcu;
use gd32vf103xx_hal::spi::{Spi, MODE_0};
use heapless::String;
use longan_nano::epd27b;
use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::{pac, prelude::*};
use longan_nano::led::{rgb, Led};
use longan_nano::{epd_pins, sprintln};
use panic_halt as _;
use profont::{ProFont12Point, ProFont14Point, ProFont24Point, ProFont9Point};
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);

    let (mut red, mut green, mut blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
    let leds: [&mut dyn Led; 3] = [&mut red, &mut green, &mut blue];

    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        57600.bps(),
        &mut afio,
        &mut rcu,
    );
    let mut delay = McycleDelay::new(&rcu.clocks);

    let pins = epd_pins!(gpioa, gpiob);

    let mut spi = Spi::spi0(
        dp.SPI0,
        (pins.sck, pins.miso, pins.mosi),
        &mut afio,
        MODE_0,
        4.mhz(),
        &mut rcu,
    );
    sprintln!("Epd2in7b init");
    let mut epd =
        Epd2in7b::new(&mut spi, pins.cs, pins.busy, pins.dc, pins.rst, &mut delay).unwrap();
    sprintln!("Epd2in7b init ok");
    sprintln!("clear frame");
    epd.clear_frame(&mut spi, &mut delay);
    sprintln!("clear frame ok");
    let mut bw = Display2in7b::default();
    bw.set_rotation(DisplayRotation::Rotate90);
    let mut red = Display2in7b::default();
    red.set_rotation(DisplayRotation::Rotate90);
    draw_text_black(&mut bw, "Hello risc-v", 1, 5);
    // epd.update_partial_frame(&mut spi, bw.buffer(), 1, 5, 100, 20)
    //     .ok();
    // epd.update_achromatic_frame(&mut spi, bw.buffer()).ok();
    draw_text_red(&mut red, "\nHello e-paper", 1, 5);
    //epd.update_partial_frame(&mut spi, red.buffer(), 1, 10, 100, 20)
    // .ok();
    // epd.update_and_display_frame(&mut spi, red.buffer(), &mut delay)
    //     .ok();
    epd.update_color_frame(&mut spi, bw.buffer(), red.buffer());
    // //epd.update_chromatic_frame(&mut spi, red.buffer()).ok();
    epd.display_frame(&mut spi, &mut delay).ok();

    let mut i = 0;
    loop {
        let inext = (i + 1) % leds.len();
        leds[i].off();
        leds[inext].on();
        delay.delay_ms(500);

        i = inext;
    }
}

fn draw_text_black(display: &mut Display2in7b, text: &str, x: i32, y: i32) {
    display.clear_buffer(Color::White);
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off)
        .build();

    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    Text::with_text_style(text, Point::new(x, y), style, text_style)
        .draw(display)
        .ok();
}
fn draw_text_red(display: &mut Display2in7b, text: &str, x: i32, y: i32) {
    display.clear_buffer(Color::White);
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();
    Text::with_text_style(text, Point::new(x, y), style, text_style)
        .draw(display)
        .ok();
}
