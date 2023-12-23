#![no_std]
#![feature(type_alias_impl_trait)]

pub mod binary_info;
pub mod frame;
pub mod panic;
pub mod secret;
pub mod ws2812;

use cyw43_pio::PioSpi;
use defmt_rtt as _;
use embassy_net::Stack;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, USB};
use embassy_rp::usb::Driver;

#[embassy_executor::task]
pub async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
pub async fn logger_task(driver: Driver<'static, USB>) {
    #[cfg(not(debug_assertions))]
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
    #[cfg(debug_assertions)]
    embassy_usb_logger::run!(1024, log::LevelFilter::Debug, driver);
}
