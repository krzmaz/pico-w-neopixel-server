//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]
#![allow(incomplete_features)]

use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::StackResources;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_time::Duration;
use pico_w_neopixel_server::ws2812::Ws2812;
use pico_w_neopixel_server::{frame, logger_task, net_task, secret, wifi_task};
use ringbuf::StaticRb;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
    let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);
    #[cfg(feature = "startup_fill")]
    {
        let warm_white = (127, 54, 14); // 2700K, 70% apparent brightness, gamma corrected
        pico_w_neopixel_server::frame::fill_lights(warm_white, 1500, &mut ws2812).await;
    }

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

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

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));
    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::None).await;
    control.gpio_set(0, true).await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);

    // Launch network task that runs `stack.run().await`
    spawner.spawn(net_task(runner)).unwrap();

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

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(1)));

        log::info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }
        log::info!("Received connection from {:?}", socket.remote_endpoint());

        cons.clear();

        loop {
            while let Some(frame_length) = frame::next_length(&cons) {
                frame::display_frame(&mut cons, frame_length, &mut ws2812).await;
            }

            match socket
                .read_with(|x| {
                    log::debug!("Received bytes: {:?}", x);
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
        }
    }
}
