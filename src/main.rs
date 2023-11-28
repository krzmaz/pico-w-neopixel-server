//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

mod panic;
use core::mem::MaybeUninit;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Executor;
use embassy_rp::multicore::Stack as MTStack;
use pio::ArrayVec;
use static_cell::StaticCell;
mod binary_info;
mod secret;
mod ws2812;
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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use itertools::Itertools;
use ringbuf::{Consumer, Producer, SharedRb, StaticRb};
use static_cell::make_static;

use crate::ws2812::Ws2812;

enum LedState {
    On,
    Off,
}

static mut CORE1_STACK: MTStack<4096> = MTStack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, LedState, 1> = Channel::new();

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

type NetworkBufferType = u8;
const NETWORK_BUFFER_SIZE: usize = 8192;
type NetworkProducerType = Producer<
    NetworkBufferType,
    &'static SharedRb<NetworkBufferType, [MaybeUninit<NetworkBufferType>; NETWORK_BUFFER_SIZE]>,
>;
type NetworkConsumerType = Consumer<
    NetworkBufferType,
    &'static SharedRb<NetworkBufferType, [MaybeUninit<NetworkBufferType>; NETWORK_BUFFER_SIZE]>,
>;

static NETWORK_RING_BUFFER: StaticCell<StaticRb<NetworkBufferType, NETWORK_BUFFER_SIZE>> = StaticCell::new();

#[embassy_executor::task]
async fn core0_main(
    driver: Driver<'static, USB>,
    pwr: Output<'static, PIN_23>,
    spi: PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    mut net_prod: NetworkProducerType,
    spawner: Spawner,
) {
    spawner.spawn(logger_task(driver)).unwrap();

    let fw = include_bytes!("../embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../embassy/cyw43-firmware/43439A0_clm.bin");

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

    let mut rx_buffer = [0; 16384];
    let mut tx_buffer = [0; 1024];

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
            let _count = match socket
                .read_with(|x| {
                    let n = net_prod.push_slice(x);
                    (n, n)
                })
                .await
            {
                Ok(0) => {
                    log::warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            };
        }
    }
}

type FrameType = ArrayVec<u32, 8192>;
const FRAME_BUFFER_SIZE: usize = 3;
type FrameProducerType = Producer<FrameType, &'static SharedRb<FrameType, [MaybeUninit<FrameType>; FRAME_BUFFER_SIZE]>>;
type FrameConsumerType = Consumer<FrameType, &'static SharedRb<FrameType, [MaybeUninit<FrameType>; FRAME_BUFFER_SIZE]>>;

static FRAMES_RING_BUFFER: StaticCell<StaticRb<FrameType, FRAME_BUFFER_SIZE>> = StaticCell::new();

#[embassy_executor::task]
async fn convert_frame(mut net_cons: NetworkConsumerType, mut frame_prod: FrameProducerType) {
    let mut num_bytes: Option<u16> = None;

    loop {
        if net_cons.len() > 1 {
            if num_bytes.is_none() {
                num_bytes = Some(u16::from_le_bytes([net_cons.pop().unwrap(), net_cons.pop().unwrap()]));
            }
            let length = num_bytes.unwrap() as usize;
            if net_cons.len() >= length {
                // there might be the start of the next "frame" in the input buffer
                let overflow = net_cons.len() - length;
                if overflow > 0 {
                    // log::warn!("overflow len: {:?}", overflow);
                }

                let f = net_cons
                    .pop_iter()
                    .take(length)
                    .tuples()
                    .map(|(r, g, b)| ((u32::from(g) << 24) | (u32::from(r) << 16) | (u32::from(b) << 8)))
                    .collect();

                let _ = frame_prod.push(f);

                num_bytes = None;
            }
        }
        Timer::after_micros(1).await;
    }
}

#[embassy_executor::task]
async fn push_frame(mut ws2812: Ws2812<'static, PIO1, 0>, mut frame_cons: FrameConsumerType) {
    loop {
        if !frame_cons.is_empty() {
            let f = frame_cons.pop().unwrap();
            ws2812.write_dma(&f).await;

            // Latch LEDs
            Timer::after_micros(60).await;
        }
        Timer::after_micros(1).await;
    }
}

#[embassy_executor::task]
async fn core1_main(ws2812: Ws2812<'static, PIO1, 0>, net_cons: NetworkConsumerType, spawner: Spawner) {
    let rb = FRAMES_RING_BUFFER.init(StaticRb::default());

    let (frame_prod, frame_cons) = rb.split_ref();

    spawner.spawn(convert_frame(net_cons, frame_prod)).unwrap();
    spawner.spawn(push_frame(ws2812, frame_cons)).unwrap();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
    let ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);

    let rb = NETWORK_RING_BUFFER.init(StaticRb::default());

    let (prod, cons) = rb.split_ref();

    embassy_rp::multicore::spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| spawner.spawn(core1_main(ws2812, cons, spawner)).unwrap());
    });

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio0 = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio0.common, pio0.sm0, pio0.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.spawn(core0_main(driver, pwr, spi, prod, spawner)).unwrap())
}

// bind_interrupts!(struct Irqs {
//     USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
//     PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
//     PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
// });

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());
//     let driver = Driver::new(p.USB, Irqs);

//     let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
//     let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);
//     spawner.spawn(logger_task(driver)).unwrap();

//     let fw = include_bytes!("../embassy/cyw43-firmware/43439A0.bin");
//     let clm = include_bytes!("../embassy/cyw43-firmware/43439A0_clm.bin");

//     // To make flashing faster for development, you may want to flash the firmwares independently
//     // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
//     //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
//     //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
//     //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
//     //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

//     let pwr = Output::new(p.PIN_23, Level::Low);
//     let cs = Output::new(p.PIN_25, Level::High);
//     let mut pio0 = Pio::new(p.PIO0, Irqs);
//     let spi = PioSpi::new(&mut pio0.common, pio0.sm0, pio0.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

//     let state = make_static!(cyw43::State::new());
//     let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
//     unwrap!(spawner.spawn(wifi_task(runner)));
//     control.init(clm).await;
//     control.set_power_management(cyw43::PowerManagementMode::None).await;
//     control.gpio_set(0, true).await;

//     let config = embassy_net::Config::dhcpv4(Default::default());
//     // Generate random seed
//     let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

//     // Init network stack
//     let stack = &*make_static!(Stack::new(
//         net_device,
//         config,
//         make_static!(StackResources::<2>::new()),
//         seed
//     ));

//     unwrap!(spawner.spawn(net_task(stack)));

//     loop {
//         // use the example file to create secret.rs
//         match control.join_wpa2(secret::WIFI_NETWORK, secret::WIFI_PASSWORD).await {
//             Ok(_) => break,
//             Err(err) => {
//                 log::info!("join failed with status={}", err.status);
//             }
//         }
//     }

//     // Wait for DHCP, not necessary when using static IP
//     log::info!("waiting for DHCP...");
//     stack.wait_config_up().await;
//     log::info!("DHCP is now up! {}", stack.config_v4().unwrap().address);

//     control.gpio_set(0, false).await;
//     // And now we can use it!

//     let mut rx_buffer = [0; 16384];
//     let mut tx_buffer = [0; 1024];
//     let mut rb = StaticRb::<u8, 16384>::default();
//     let (mut prod, mut cons) = rb.split_ref();
//     let mut num_bytes: Option<u16> = None;

//     loop {
//         let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
//         socket.set_timeout(Some(Duration::from_secs(10)));
//         socket.set_keep_alive(Some(Duration::from_secs(10)));

//         log::info!("Listening on TCP:1234...");
//         if let Err(e) = socket.accept(1234).await {
//             log::warn!("accept error: {:?}", e);
//             continue;
//         }

//         log::info!("Received connection from {:?}", socket.remote_endpoint());

//         loop {
//             let count = match socket
//                 .read_with(|x| {
//                     let n = prod.push_slice(x);
//                     (n, n)
//                 })
//                 .await
//             {
//                 Ok(0) => {
//                     log::warn!("read EOF");
//                     break;
//                 }
//                 Ok(n) => n,
//                 Err(e) => {
//                     log::warn!("read error: {:?}", e);
//                     break;
//                 }
//             };

//             if count > 1 {
//                 if num_bytes.is_none() {
//                     num_bytes = Some(u16::from_le_bytes([cons.pop().unwrap(), cons.pop().unwrap()]));
//                 }
//                 let length = num_bytes.unwrap() as usize;
//                 if cons.len() >= length {
//                     // there might be the start of the next "frame" in the input buffer
//                     let overflow = cons.len() - length;
//                     if overflow > 0 {
//                         log::warn!("overflow len: {:?}", overflow);
//                     }
//                     cons.pop_iter()
//                         .take(length)
//                         .tuples()
//                         .for_each(|(r, g, b)| ws2812.write(r, g, b));

//                     num_bytes = None;
//                     // let the neopixels latch on
//                     Timer::after_micros(60).await;
//                 }
//             }
//         }
//     }
// }
