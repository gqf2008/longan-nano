#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::{pac, prelude::*};
use longan_nano::heap::RISCVHeap;
use longan_nano::led::{rgb, Led};
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
    let mut rcu = dp.RCU.configure().freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);
    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);
    let (mut red, mut green, mut blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
    let leds: [&mut dyn Led; 3] = [&mut red, &mut green, &mut blue];

    let mut delay = McycleDelay::new(&rcu.clocks);

    let mut i = 0;
    loop {
        let inext = (i + 1) % leds.len();
        leds[i].off();
        leds[inext].on();
        delay.delay_ms(1000);

        i = inext;
    }
}

// 内存不足执行此处代码(调试用)
#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    loop {}
}
