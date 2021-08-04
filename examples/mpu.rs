#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
use alloc::format;
use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::{ascii::FONT_10X20, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};

use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_hal::blocking::delay::DelayUs;
use heapless::Vec;
use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::gpio::*;
use longan_nano::hal::i2c::{BlockingI2c, DutyCycle, Mode};
use longan_nano::hal::{pac, prelude::*};
use longan_nano::hcsr04::HcSr04;
use longan_nano::heap::RISCVHeap;
use longan_nano::{lcd, lcd_pins, sprintln};
use mpu6050::*;
use nalgebra::{matrix, vector};
use panic_halt as _;
use riscv_rt::entry;

#[global_allocator]
static ALLOCATOR: RISCVHeap = RISCVHeap::empty();

/// 堆内存 16K
const HEAP_SIZE: usize = 8192 * 2;

fn init() {
    unsafe {
        ALLOCATOR.init(heap_start() as usize, HEAP_SIZE);
    }
}

#[inline]
pub fn heap_start() -> *mut u32 {
    extern "C" {
        static mut _sheap: u32;
    }

    unsafe { &mut _sheap }
}

#[entry]
fn main() -> ! {
    init();
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    let mut delay = McycleDelay::new(&rcu.clocks);
    let mut afio = dp.AFIO.constrain(&mut rcu);
    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        57600.bps(),
        &mut afio,
        &mut rcu,
    );
    sprintln!("start {:#08x}", heap_start() as usize);
    let mut trig = gpioa.pa0.into_push_pull_output_with_state(State::Low);
    let echo = gpioa.pa1.into_pull_down_input(); // 下拉输入
    let mut sensor = HcSr04::new(trig, echo, delay, rcu.clocks.sysclk().0 as u64);
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();
    let i2c = BlockingI2c::i2c0(
        dp.I2C0,
        (scl, sda),
        &mut afio,
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &mut rcu,
        1000,
        10,
        1000,
        1000,
    );

    sprintln!("init mpu6050");
    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).ok();
    sprintln!("init mpu6050 ok");
    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    lcd.set_orientation(&st7735_lcd::Orientation::LandscapeSwapped);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    // Clear screen
    Rectangle::new(Point::new(0, 0), Size::new(width as u32, height as u32))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut lcd)
        .unwrap();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    loop {
        // get roll and pitch estimate
        let rp = mpu.get_acc_angles().unwrap();
        // sprintln!("r/p: {:?}", rp);

        // // get temp
        // let temp = mpu.get_temp().unwrap();
        // sprintln!("temp: {:?}c", temp);

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        //sprintln!("gyro: {:?}", gyro);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();
        //sprintln!("acc: {:?}", acc);

        let distance = sensor.measure();
        Text::new(
            format!(
                " {:.2}cm\n R/P:{:?}\n G:{:?} A:{:?}",
                distance.cm(),
                rp,
                gyro,
                acc
            )
            .as_str(),
            Point::new(0, 10),
            style,
        )
        .draw(&mut lcd)
        .unwrap();
    }
}

// 内存不足执行此处代码(调试用)
#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    loop {}
}
