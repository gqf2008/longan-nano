#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
use alloc::format;
use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::blocking::delay::DelayUs;
use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::gpio::*;
use longan_nano::hal::i2c::{BlockingI2c, DutyCycle, Mode};
use longan_nano::hal::{pac, prelude::*};
use longan_nano::hcsr04::HcSr04;
use longan_nano::heap::RISCVHeap;
use longan_nano::{lcd, lcd_pins, sprintln};
use panic_halt as _;
use riscv_rt::entry;
use ssd1306::prelude::*;
use ssd1306::*;

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

    // Configure clocks
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
    let mut trig = gpiob.pb10.into_push_pull_output_with_state(State::Low);
    let echo = gpiob.pb11.into_pull_down_input(); // 下拉输入
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
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();
    let (w, h) = display.dimensions();
    let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("./sqb.raw"), 120);

    Image::new(&raw, Point::new(4, 1))
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    loop {
        // led.toggle();
        let distance = sensor.measure();
        //sprintln!("D {:.2}cm", distance.cm());
        display.clear();
        Image::new(&raw, Point::new(4, 1))
            .draw(&mut display)
            .unwrap();
        Text::with_alignment(
            format!("D {:.2}cm", distance.cm()).as_str(),
            Point::new(64, 60),
            style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();
        // // writeln!(stdout, "距离:{}毫米", distance.mm());
        // nb::block!(tim.wait()).ok();
    }
}

// 内存不足执行此处代码(调试用)
#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    loop {}
}
