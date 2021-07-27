#![no_std]
#![no_main]

use panic_halt as _;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;

use longan_nano::hal::{pac, prelude::*};
use longan_nano::{lcd, lcd_pins};

use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);

    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    // Clear screen
    Rectangle::new(Point::new(0, 0), Point::new(width - 1, height - 1))
        .into_styled(gb565::BLACK)
        .draw(&mut lcd)
        .unwrap();

    // let style = text_style!(
    //     font = Font12x16,
    //     text_color = Rgb565::BLACK,
    //     background_color = Rgb565::GREEN
    // );

    // Create a text at position (20, 30) and draw it using style defined above
    Text::new(" Austin ", Point::new(20, 18))
        .into_styled(style)
        .draw(&mut lcd)
        .unwrap();

    loop {}
}
