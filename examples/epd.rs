#![no_std]
#![no_main]

// use embedded_graphics::{
//     pixelcolor::BinaryColor::On as Black, prelude::*, primitives::Line, primitives::PrimitiveStyle,
//     text::Text,
// };
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
use epd_waveshare::{epd2in7b::Display2in7b, prelude::*};
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

    let mut epd = epd27b::EPD27b::new(spi, pins.cs, pins.dc, pins.rst, pins.busy, delay);
    epd.init();
    sprintln!("Epd2in7b init ok");
    sprintln!("clear white");
    epd.clear_white();
    sprintln!("clear white ok");
    let mut frame = Display2in7b::default();
    frame.set_rotation(DisplayRotation::Rotate90);
    draw_text_black(&mut frame, "Hello Epaper", 0, 5);
    epd.send_black(frame.buffer());
    draw_text_red(&mut frame, "\nHello Austin\nHello laopo", 0, 5);
    epd.send_red(frame.buffer());

    epd.display();

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
    display.clear_buffer(Color::Black);
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_9X18_BOLD)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();
    Text::with_text_style(text, Point::new(x, y), style, text_style)
        .draw(display)
        .ok();
}
