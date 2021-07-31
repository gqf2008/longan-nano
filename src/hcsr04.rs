//!超声波测距传感器

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

#[derive(Debug, Copy, Clone)]
pub struct Distance(f64);

impl Distance {
    pub fn cm(&self) -> f64 {
        self.0 / 10.0
    }
    pub fn mm(&self) -> f64 {
        self.0
    }
}

pub struct HcSr04<Triger, Echo, Delay> {
    trig: Triger,
    echo: Echo,
    delay: Delay,
    timer: MonoTimer,
}

impl<Trig, Echo, Delay> HcSr04<Trig, Echo, Delay>
where
    Trig: OutputPin,
    Echo: InputPin,
    Delay: DelayUs<u32>,
{
    pub fn new(mut trig: Trig, echo: Echo, delay: Delay, core_frequency: u64) -> Self {
        trig.set_low().ok();
        HcSr04 {
            trig,
            echo,
            delay,
            timer: MonoTimer {
                frequency: core_frequency,
            },
        }
    }

    pub fn measure(&mut self) -> Distance {
        self.delay.delay_us(50000u32);
        let sum = self.measure1();
        Distance(sum)
    }

    fn measure1(&mut self) -> f64 {
        //发送信号
        self.trig.set_high().ok();
        self.delay.delay_us(20u32);
        self.trig.set_low().ok();
        // let t0 = riscv::register::mcycle::read64();
        // let clocks = (us * (self.core_frequency as u64)) / 1_000_000;
        // while riscv::register::mcycle::read64().wrapping_sub(t0) <= clocks {}
        //let start_wait = self.timer.now();
        //等高电平
        while let Ok(true) = self.echo.is_low() {
            // if start_wait.elapsed() > self.timer.frequency() * 5 {
            //     return Err(Error::Timeout);
            // }
        }
        //等低电平（高电平持续的时间就是信号往返的时间）
        let start_instant = self.timer.now();
        //crate::sprintln!("start {}", start_instant.now);
        while let Ok(true) = self.echo.is_high() {
            // if start_instant.elapsed() > self.timer.frequency().0 {
            //     return Err(Error::Timeout);
            // }
        }
        let ticks = start_instant.elapsed() as f64;
        // crate::sprintln!("elapsed {}", ticks);
        ticks / self.timer.frequency() as f64 * 170.0 * 1000.0
    }
}

#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: u64,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(frequency: u64) -> Self {
        MonoTimer { frequency }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(self) -> u64 {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
        Instant {
            now: riscv::register::mcycle::read64(),
        }
    }
}

/// A measurement of a monotonically non-decreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u64,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u64 {
        riscv::register::mcycle::read64().wrapping_sub(self.now)
    }
}
