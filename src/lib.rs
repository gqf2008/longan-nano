//! Board support crate for the Longan Nano board

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub use gd32vf103xx_hal as hal;

pub mod epd27b;
pub mod hcsr04;
pub mod heap;
pub mod lcd;
pub mod led;
pub mod stdout;
