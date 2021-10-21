#![no_std]
#![no_main]
#![feature(llvm_asm)]
#![feature(alloc_error_handler)]

use panic_halt as _;

// use gd32vf103_rt as rt;
// use gd32vf103_pac as pac;
use gd32vf103xx_hal as hal;
use hal::pac;
use hal::{
    backup_domain::BkpExt,
    eclic::{EclicExt, Level, LevelPriorityBits, Priority, TriggerType},
    gpio::GpioExt,
    prelude::*,
    rcu::RcuExt,
    rtc::Rtc,
    signature,
    timer::{Event, Timer},
};
use longan_nano::heap::RISCVHeap;
use longan_nano::led::{rgb, Led};
use longan_nano::sprintln;
use pac::{Interrupt, ECLIC, RTC, TIMER0};
use riscv::interrupt::Nr;
use riscv_rt as rt;

static mut G_TIMER0: Option<Timer<TIMER0>> = None;

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
#[rt::entry]
fn main() -> ! {
    init();
    let mut dp = pac::Peripherals::take().unwrap();
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

    ECLIC::setup(Interrupt::RTC, TriggerType::Level, Level::L7, Priority::P1);
    ECLIC::setup(
        Interrupt::TIMER0_UP,
        TriggerType::Level,
        Level::L0,
        Priority::P0,
    );
    unsafe { ECLIC::unmask(Interrupt::RTC) };
    unsafe { ECLIC::unmask(Interrupt::TIMER0_UP) };

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);

    let (mut red, mut green, mut blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
    red.off();
    green.off();
    blue.off();

    let mut afio = dp.AFIO.constrain(&mut rcu);
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        57600.bps(),
        // 921_600.bps(),
        &mut afio,
        &mut rcu,
    );

    sprintln!(
        "Starting [debug_id={:#08X}, flash_size: {}KB, sram_size={}KB]",
        dp.DBG.id.read().bits(),
        signature::flash_size_kb(),
        signature::sram_size_kb()
    );

    let mut backup_domain = dp.BKP.configure(&mut rcu, &mut dp.PMU);
    let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);
    rtc.set_time(0);
    rtc.listen_seconds();
    rtc.clear_second_flag();

    // Do _not_ try print trap mode here. It's set to value unahndled by current crate.
    sprintln!(
        "mtvec [address={:#08X}]",
        riscv::register::mtvec::read().address()
    );
    sprintln!(
        "ECLIC [ctl_bits={}, version={}, num_interrupt={}, level_bits={}, priority_bits={}]",
        dp.ECLIC.clicinfo.read().clicintctlbits().bits(),
        dp.ECLIC.clicinfo.read().version().bits(),
        dp.ECLIC.clicinfo.read().num_interrupt().bits(),
        ECLIC::get_level_bits(),
        ECLIC::get_priority_bits(),
    );
    let nr = Interrupt::RTC.nr() as usize;
    sprintln!(
        "ECLIC[RTC] [nlbits={}, clicintie={:#X}, clicintip={:#X}, clicintctl={:#X}, clicintattr={:#X}, level={:?}, priority={:?}]",
        dp.ECLIC.cliccfg.read().nlbits().bits(),
        dp.ECLIC.clicints[nr].clicintie.read().bits(),
        dp.ECLIC.clicints[nr].clicintip.read().bits(),
        dp.ECLIC.clicints[nr].clicintctl.read().bits(),
        dp.ECLIC.clicints[nr].clicintattr.read().bits(),
        ECLIC::get_level(Interrupt::RTC),
        ECLIC::get_priority(Interrupt::RTC),
    );

    let mut timer = Timer::timer0(dp.TIMER0, 2.hz(), &mut rcu);
    timer.listen(Event::Update);
    unsafe { G_TIMER0 = Some(timer) };

    unsafe { riscv::interrupt::enable() };

    loop {
        unsafe {
            llvm_asm!("csrci 0x811, 1");
            // llvm_asm!("csrci 0xFFF, 1");
            riscv::asm::wfi()
        };

        rtc.clear_second_flag();
        sprintln!("RTC {}", rtc.current_time());
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn TIMER0_UP() {
    sprintln!("TIMER0_UP IRQ");

    if let Some(ref mut timer) = unsafe { &mut G_TIMER0 } {
        timer.clear_update_interrupt_flag();
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn RTC() {
    sprintln!("RTC IRQ");

    // Hack
    let rtc = unsafe { &*RTC::ptr() };
    rtc.ctl.write(|w| w.scif().clear_bit());
}

#[allow(non_snake_case)]
#[no_mangle]
fn ExceptionHandler(_: &rt::TrapFrame) {
    let cause =
        riscv::register::mcause::Exception::from(riscv::register::mcause::read().code() & 0xFFF);

    sprintln!("ExceptionHandler [cause={:?}]", cause);

    loop {}
}

#[allow(non_snake_case)]
#[no_mangle]
fn DefaultHandler() {
    let code = riscv::register::mcause::read().code() & 0xFFF;
    let cause = riscv::register::mcause::Exception::from(code);

    sprintln!("DefaultHandler [code={}, cause={:?}]", code, cause);

    loop {}
}

// 内存不足执行此处代码(调试用)
#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    loop {}
}
