#![no_std]
#![no_main]
#![feature(llvm_asm)]
#![feature(alloc_error_handler)]

extern crate alloc;

use core::fmt::Write;

use alloc::format;
use gd32vf103xx_hal::pac::USART0;
use panic_halt as _;

use longan_nano::hal::delay::McycleDelay;
use longan_nano::hal::eclic::{EclicExt, Level, LevelPriorityBits, Priority, TriggerType};
use longan_nano::hal::pac::{Interrupt, ECLIC};
use longan_nano::hal::serial::*;
use longan_nano::hal::{pac, prelude::*};
use longan_nano::heap::RISCVHeap;
use longan_nano::led::{rgb, Led};
use longan_nano::{lcd, lcd_pins, sprintln};
use riscv::interrupt::{Mutex, Nr};
use riscv_rt::entry;

#[global_allocator]
static ALLOCATOR: RISCVHeap = RISCVHeap::empty();

static mut TX: Option<Tx<USART0>> = None;
static mut RX: Option<Rx<USART0>> = None;

/// 堆内存 16K
const HEAP_SIZE: usize = 8192 * 2;

macro_rules ! interrupt {
    ($NAME:ident, $path:path, locals: {$($lvar:ident: $lty:ty = $lval:expr;)*}) => {
         #[allow(non_snake_case)]
         mod $NAME{pub struct Locals{$(pub $lvar: $lty,)*}}
        #[allow(non_snake_case)]
        #[no_mangle]
        pub extern "C" fn $NAME() {
            let _ = gd32vf103xx_hal::pac::Interrupt::$NAME ;
            static mut LOCALS:self::$NAME::Locals =self::$NAME::Locals {$($lvar: $lval,)*};
            let f: fn (&mut self::$NAME::Locals) = $path ;
            f(unsafe {&mut LOCALS});
         }
    };
    ($NAME:ident,$path:path) => {
         #[allow(non_snake_case)]
         #[no_mangle]
         pub extern "C" fn $NAME() {
             let _ = gd32vf103xx_hal::pac::Interrupt::$NAME;
             let f : fn() = $path ;
            f();
         }
    }
}

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
    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    // Use 3 bits for level, 1 for priority
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);
    ECLIC::setup(
        Interrupt::USART0,
        TriggerType::Level,
        Level::L0,
        Priority::P0,
    );
    unsafe {
        ECLIC::unmask(Interrupt::USART0);
    }
    let mut afio = dp.AFIO.constrain(&mut rcu);
    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);
    let (mut red, mut green, mut blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
    let leds: [&mut dyn Led; 3] = [&mut red, &mut green, &mut blue];
    let mut delay = McycleDelay::new(&rcu.clocks);
    let tx = gpioa.pa9.into_alternate_push_pull();
    let rx = gpioa.pa10.into_floating_input();
    let config = Config {
        baudrate: 57600.bps(),
        parity: Parity::ParityNone,
        stopbits: StopBits::STOP1,
    };
    let serial = Serial::new(dp.USART0, (tx, rx), config, &mut afio, &mut rcu);
    let (mut tx, mut rx) = serial.split();
    tx.listen();
    rx.listen();
    rx.listen_idle();
    riscv::interrupt::free(|_| unsafe {
        TX.replace(tx);
        RX.replace(rx);
    });
    interrupt!(USART0, usart_interrupt);

    unsafe {
        riscv::interrupt::enable();
    }
    let mut i = 0;
    loop {
        unsafe {
            llvm_asm!("csrci 0x811, 1");
            // llvm_asm!("csrci 0xFFF, 1");
            riscv::asm::wfi();
        };
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

unsafe fn write_buffer() {
    BUFFER[0..IDX].iter().for_each(|w| {
        let tx = TX.as_mut().unwrap();
        while !tx.is_txe() {}
        nb::block!(tx.write(*w)).unwrap();
    });
    IDX = 0;
}

static mut BUFFER: &mut [u8; 64] = &mut [0; 64];
static mut IDX: usize = 0;
static mut IDLE: bool = false;
fn usart_interrupt() {
    riscv::interrupt::free(|_cs| unsafe {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rxne() {
                match nb::block!(rx.read()) {
                    Ok(w) => {
                        // TX.as_mut().unwrap().write(w);
                        BUFFER[IDX] = w;
                        IDX += 1;
                        if IDX == 64 {
                            write_buffer();
                            IDX = 0;
                        }
                        IDLE = false;
                    }
                    Err(e) => {
                        IDLE = true;
                        TX.as_mut()
                            .unwrap()
                            .write_str(format!("{:?}\n", e).as_str());
                    }
                }
            } else if rx.is_idle() {
                if !IDLE {
                    write_buffer();
                    IDLE = true;
                }
            }
        }
    });
}
