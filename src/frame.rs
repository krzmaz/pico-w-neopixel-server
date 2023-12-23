use embassy_rp::pio::Instance;
use embassy_time::Timer;
use itertools::Itertools;
use ringbuf::ring_buffer::{RbRead, RbRef};
use ringbuf::Consumer;

use crate::ws2812::Ws2812;

fn peek<T, R>(cons: &Consumer<T, R>, index: usize) -> Option<&T>
where
    R: RbRef,
    R::Rb: RbRead<T>,
{
    let (left, right) = cons.as_slices();
    if index < left.len() {
        Some(&left[index])
    } else if index < left.len() + right.len() {
        Some(&right[index - left.len()])
    } else {
        None
    }
}

pub fn next_length<R>(cons: &Consumer<u8, R>) -> Option<usize>
where
    R: RbRef,
    R::Rb: RbRead<u8>,
{
    peek(cons, 0)
        .zip(peek(cons, 1))
        .map(|(l1, l2)| u16::from_le_bytes([*l1, *l2]))
        .map(|l| l as usize)
        .filter(|len| *len < cons.len())
}

pub async fn display_frame<R, P, const S: usize>(cons: &mut Consumer<u8, R>, len: usize, ws2812: &mut Ws2812<'_, P, S>)
where
    R: RbRef,
    R::Rb: RbRead<u8>,
    P: Instance,
{
    for (r, g, b) in cons.pop_iter().skip(2).take(len).tuples() {
        ws2812.write(r, g, b);
    }

    // let the neopixels latch on
    Timer::after_micros(60).await;
}
