#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use bbqueue::framed::{FrameConsumer, FrameGrantW, FrameProducer};
use bbqueue::BBBuffer;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::Stack as MCStack;
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use heapless::Vec;
use itertools::Itertools;
use pico_w_neopixel_server::ws2812::{convert_colors_to_word, Ws2812};
use pico_w_neopixel_server::{logger_task, net_task, secret, wifi_task};
use static_cell::{make_static, StaticCell};
use {defmt_rtt as _, panic_probe as _};

const MAX_PIXELS_IN_FRAME: usize = 1024;
// frame buffer stores u32, which is 4 bytes per pixel
const FRAME_BUFFER_MEMORY: usize = (4 * MAX_PIXELS_IN_FRAME) * FRAME_BUFFER_SIZE;
// const CORE1_STACK_SIZE: usize = FRAME_BUFFER_MEMORY * 4;
static mut CORE1_STACK: MCStack<8192> = MCStack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, u16, FRAME_BUFFER_SIZE> = Channel::new();
static BB: BBBuffer<FRAME_BUFFER_MEMORY> = BBBuffer::new();

type ConsumerType = FrameConsumer<'static, FRAME_BUFFER_MEMORY>;
type ProducerType = FrameProducer<'static, FRAME_BUFFER_MEMORY>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

type NetworkBufferType = u8;
const NETWORK_BUFFER_SIZE: usize = 3 * MAX_PIXELS_IN_FRAME * 2;
// type NetworkProducerType = Producer<'static, NetworkBufferType, NETWORK_BUFFER_SIZE>;
// type NetworkConsumerType = Consumer<'static, NetworkBufferType, NETWORK_BUFFER_SIZE>;

// static NETWORK_RING_BUFFER: StaticCell<StaticRb<NetworkBufferType, NETWORK_BUFFER_SIZE>> = StaticCell::new();

type FrameType = Vec<u32, MAX_PIXELS_IN_FRAME>;
const FRAME_BUFFER_SIZE: usize = 4;
// type FrameProducerType = Producer<'static, FrameType, FRAME_BUFFER_SIZE>;
// type FrameConsumerType = Consumer<'static, FrameType, FRAME_BUFFER_SIZE>;

// static FRAMES_RING_BUFFER: StaticCell<Queue<FrameType, FRAME_BUFFER_SIZE>> = StaticCell::new();

#[embassy_executor::task]
async fn core0_main(
    driver: Driver<'static, USB>,
    pwr: Output<'static, PIN_23>,
    spi: PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    mut prod: ProducerType,
    spawner: Spawner,
) {
    spawner.spawn(logger_task(driver)).unwrap();

    let fw = include_bytes!("../../embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../embassy/cyw43-firmware/43439A0_clm.bin");

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
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    stack.wait_config_up().await;
    info!("DHCP is now up!");

    control.gpio_set(0, false).await;

    let mut rx_buffer = [0; NETWORK_BUFFER_SIZE];
    let mut tx_buffer = [0; 1024];
    let mut converter = NetworkDataConverter::new(prod);
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket.set_keep_alive(Some(Duration::from_secs(10)));

        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            match socket
                .read_with(|x| {
                    info!("Writing: {} bytes", x.len());
                    let written = converter.push(x);
                    info!("Written {} bytes", written);
                    (written, x.is_empty())
                })
                .await
            {
                Ok(true) => {
                    warn!("read EOF");
                    break;
                }
                Ok(_) => (),
                Err(e) => {
                    // warn!("read error");
                    // log::warn!("read error: {:?}", e);
                    break;
                }
            };
        }
    }
}

// #[embassy_executor::task]
// async fn convert_frame(mut net_cons: NetworkConsumerType, mut frame_prod: FrameProducerType) {
//     let mut num_bytes: Option<u16> = None;

//     loop {
//         if net_cons.len() > 1 {
//             log::info!("Core1: Converting frame!");
//             if num_bytes.is_none() {
//                 num_bytes = Some(u16::from_le_bytes([
//                     net_cons.dequeue().unwrap(),
//                     net_cons.dequeue().unwrap(),
//                 ]));
//             }
//             let length = num_bytes.unwrap() as usize;
//             if net_cons.len() >= length {
//                 // there might be the start of the next "frame" in the input buffer
//                 let overflow = net_cons.len() - length;
//                 if overflow > 0 {
//                     // warn!("overflow len: {:?}", overflow);
//                 }

//                 let f = (0..length)
//                     .map(|_| net_cons.dequeue().unwrap())
//                     .tuples()
//                     .map(|(r, g, b)| ((u32::from(g) << 24) | (u32::from(r) << 16) | (u32::from(b) << 8)))
//                     .collect();
//                 let _ = frame_prod.enqueue(f);

//                 num_bytes = None;
//             }
//         }
//         Timer::after_micros(1000).await;
//     }
// }

// #[embassy_executor::task]
// async fn push_frame(mut ws2812: Ws2812<'static, PIO1, 0>, mut frame_cons: FrameConsumerType) {
//     loop {
//         if frame_cons.len() > 0 {
//             info!("Writing frame!");
//             let f = frame_cons.dequeue().unwrap();
//             ws2812.write_dma(&f).await;

//             // Latch LEDs
//             Timer::after_micros(60).await;
//         }
//         Timer::after_micros(1000).await;
//     }
// }

#[embassy_executor::task]
async fn core1_main(mut ws2812: Ws2812<'static, PIO1, 0>, mut cons: ConsumerType, spawner: Spawner) {
    loop {
        if let Some(data) = cons.read() {
            // data.iter().tuples().for_each(|(_, &r, &g, &b)| ws2812.write(r, g, b));
            info!("Data as ptr: {}", data.as_ptr());
            info!("Data slice as ptr: {}", data[2..].as_ptr());
            {
                let words: &[u32] =
                    unsafe { core::slice::from_raw_parts(data[2..].as_ptr() as *const _, data.len() / 4) };
                info!("Words length: {}", words.len());
                info!("Words as ptr: {}", words.as_ptr());
                ws2812.write_dma(words).await;
            }
            Timer::after_micros(60).await;
            data.release();
        } else {
            Timer::after_micros(10).await;
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
    let ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);

    let (mut prod, mut cons) = BB.try_split_framed().unwrap();

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

struct NetworkDataConverter {
    prod: ProducerType,
    // length of the current frame
    frame_len: usize,
    write_index: usize,
    frame: Option<FrameGrantW<'static, FRAME_BUFFER_MEMORY>>,
    partial_length: Option<u8>,
    first_partial_byte: Option<u8>,
    second_partial_byte: Option<u8>,
}

impl NetworkDataConverter {
    fn new(prod: ProducerType) -> Self {
        Self {
            prod,
            frame_len: 0,
            write_index: 0,
            frame: None,
            partial_length: None,
            first_partial_byte: None,
            second_partial_byte: None,
        }
    }
    fn push(&mut self, data: &[u8]) -> usize {
        if data.is_empty() {
            return 0;
        }
        let mut read = 0;
        if let Some(byte) = self.partial_length {
            self.frame_len = (u16::from_le_bytes([byte, data[0]]) as usize / 3) * 4 + 2;
            let mut frame = self.prod.grant(self.frame_len).unwrap();
            frame[0] = 0;
            frame[1] = 0;
            self.write_index = 2;
            self.frame = Some(frame);
            self.partial_length = None;
            read += 1;
            if data.len() == read {
                return read;
            }
        }
        if self.frame_len == 0 {
            if data.len() == 1 {
                self.partial_length = Some(data[0]);
                return 1;
            }
            self.frame_len = (u16::from_le_bytes([data[0], data[1]]) as usize / 3) * 4 + 2;

            info!("getting a grant with length: {}", self.frame_len);
            let mut frame = self.prod.grant(self.frame_len).unwrap();
            frame[0] = 0;
            frame[1] = 0;
            self.write_index = 2;
            self.frame = Some(frame);
            read += 2;
            if data.len() == read {
                return read;
            }
        }

        let mut data = data[read..].iter();
        if self.first_partial_byte.is_some() {
            info!("Handling partial bytes");
            if self.second_partial_byte.is_none() {
                self.second_partial_byte = Some(*data.next().unwrap());
                read += 1;
                if data.len() == 0 {
                    return read;
                }
            }
            let [a, b, c, d] = convert_colors_to_word(
                self.first_partial_byte.unwrap(),
                self.second_partial_byte.unwrap(),
                *data.next().unwrap(),
            )
            .to_le_bytes();
            self.first_partial_byte = None;
            self.second_partial_byte = None;
            {
                let frame = self.frame.as_mut().unwrap();
                frame[self.write_index] = a;
                frame[self.write_index + 1] = b;
                frame[self.write_index + 2] = c;
                frame[self.write_index + 3] = d;
            }
            self.write_index += 4;
            if self.write_index == self.frame_len {
                self.frame.take().unwrap().commit(self.frame_len);
                self.frame_len = 0;
                self.write_index = 0;
            }
            read += 1;
            if data.len() == 0 {
                return read;
            }
        }
        if self.frame_len == 0 {
            if data.len() == 1 {
                self.partial_length = Some(*data.next().unwrap());
                return read + 1;
            }
            self.frame_len = (u16::from_le_bytes([*data.next().unwrap(), *data.next().unwrap()]) as usize / 3) * 4 + 2;
            let mut frame = self.prod.grant(self.frame_len).unwrap();
            frame[0] = 0;
            frame[1] = 0;
            self.write_index = 2;
            self.frame = Some(frame);
            read += 2;
            if data.len() == 0 {
                return read;
            }
        }
        let mut chunks = data.as_slice().chunks(3);
        let mut frame = self.frame.take().unwrap();
        for chunk in chunks.by_ref() {
            if chunk.len() == 3 {
                let [a, b, c, d] = convert_colors_to_word(chunk[0], chunk[1], chunk[2]).to_le_bytes();
                frame[self.write_index] = a;
                frame[self.write_index + 1] = b;
                frame[self.write_index + 2] = c;
                frame[self.write_index + 3] = d;
                self.write_index += 4;
                read += 3;
            } else {
                // consume the unaligned data if there is some
                for byte in chunk {
                    read += 1;
                    if self.first_partial_byte.is_none() {
                        self.first_partial_byte = Some(*byte);
                        continue;
                    }
                    if self.second_partial_byte.is_none() {
                        self.second_partial_byte = Some(*byte);
                        continue;
                    }
                }
                break;
            }
            if self.write_index == self.frame_len {
                break;
            }
        }
        if self.write_index == self.frame_len {
            frame.commit(self.frame_len);
            self.frame = None;
            self.frame_len = 0;
            self.write_index = 0;
        } else {
            self.frame = Some(frame);
        }
        read
    }
}
