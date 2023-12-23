//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Timer};
use itertools::Itertools;
use pico_w_neopixel_server::secret;
use pico_w_neopixel_server::ws2812::Ws2812;
use ringbuf::StaticRb;
use static_cell::make_static;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
    let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);
    spawner.spawn(logger_task(driver)).unwrap();

    let fw = include_bytes!("../../embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio0 = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio0.common, pio0.sm0, pio0.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));
    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::None).await;
    control.gpio_set(0, true).await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    loop {
        // use the example file to create secret.rs
        match control.join_wpa2(secret::WIFI_NETWORK, secret::WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                log::info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    log::info!("waiting for DHCP...");
    stack.wait_config_up().await;
    log::info!("DHCP is now up! {}", stack.config_v4().unwrap().address);

    control.gpio_set(0, false).await;
    // And now we can use it!

    let mut rx_buffer = [0; 16384];
    let mut tx_buffer = [0; 1024];
    let mut rb = StaticRb::<u8, 16384>::default();
    let (mut prod, mut cons) = rb.split_ref();
    let mut num_bytes: Option<u16> = None;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket.set_keep_alive(Some(Duration::from_secs(10)));

        log::info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            match socket
                .read_with(|x| {
                    let n = prod.push_slice(x);
                    (n, x.is_empty())
                })
                .await
            {
                Ok(true) => {
                    log::warn!("read EOF");
                    break;
                }
                Ok(_) => (),
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            }

            if cons.len() > 1 {
                if num_bytes.is_none() {
                    num_bytes = Some(u16::from_le_bytes([cons.pop().unwrap(), cons.pop().unwrap()]));
                }
                let length = num_bytes.unwrap() as usize;
                if cons.len() >= length {
                    // there might be the start of the next "frame" in the input buffer
                    let overflow = cons.len() - length;
                    if overflow > 0 {
                        log::warn!("overflow len: {:?}", overflow);
                    }
                    cons.pop_iter()
                        .take(length)
                        .tuples()
                        .for_each(|(r, g, b)| ws2812.write(r, g, b));

                    num_bytes = None;
                    // let the neopixels latch on
                    Timer::after_micros(60).await;
                }
            }
        }
    }
}
