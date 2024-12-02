#![no_std]
#![allow(async_fn_in_trait)]

pub mod binary_info;
pub mod frame;
pub mod panic;
pub mod secret;
pub mod ws2812;

use cyw43_pio::PioSpi;
use defmt_rtt as _;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::usb::Driver;

#[embassy_executor::task]
pub async fn wifi_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn logger_task(driver: Driver<'static, USB>) {
    #[cfg(not(debug_assertions))]
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
    #[cfg(debug_assertions)]
    embassy_usb_logger::run!(1024, log::LevelFilter::Debug, driver);
}
