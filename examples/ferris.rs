#![no_std]
#![no_main]

use panic_halt as _;

use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::pixelcolor::raw::LittleEndian;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;

use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::{pac, prelude::*};
use longan_nano::{lcd, lcd_pins, sprintln};
use riscv_rt::entry;

const FERRIS: &[u8] = include_bytes!("ferris.raw");

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
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        115_200.bps(),
        &mut afio,
        &mut rcu,
    );

    let mut delay = McycleDelay::new(&rcu.clocks);
    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    // // Clear screen
    // Rectangle::new(Point::new(0, 0), Point::new(width - 1, height - 1))
    //     .into_styled(primitive_style!(fill_color = Rgb565::BLACK))
    //     .draw(&mut lcd)
    //     .unwrap();

    // Load Image Data
    let raw_image: ImageRaw<Rgb565, LittleEndian> = ImageRaw::new(&FERRIS, 86);
    let mut x = width / 2 - 43;
    sprintln!("LCD Display");
    loop {
        //lcd.clear(Rgb565::BLUE);
        // delay.delay_ms(50);
        // Clear screen
        lcd.clear(Rgb565::CYAN).ok();
        // Rectangle::new(Point::new(0, 0), Point::new(width - 1, height - 1))
        //     .into_styled(primitive_style!(fill_color = Rgb565::BLACK))
        //     .draw(&mut lcd)
        //     .unwrap();
        Image::new(&raw_image, Point::new(x, height / 2 - 32))
            .draw(&mut lcd)
            .unwrap();
        x -= 1;
        if x == 0 {
            x = width / 2 - 10;
        }
        delay.delay_ms(110);
    }
}
