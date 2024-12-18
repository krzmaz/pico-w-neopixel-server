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
        .map(|l| {
            // If we seem to have received a frame which length is not divisible
            // by 3, the only guaranteed way to recover is to panic and restart.
            assert!(l % 3 == 0);
            l as usize
        })
        // Only return Some if cons contains the full frame: at least len bytes
        // for subpixel values, plus 2 bytes for the length itself.
        .filter(|len| *len + 2 <= cons.len())
}

pub async fn display_frame<R, P, const S: usize>(cons: &mut Consumer<u8, R>, len: usize, ws2812: &mut Ws2812<'_, P, S>)
where
    R: RbRef,
    R::Rb: RbRead<u8>,
    P: Instance,
{
    // Pop the first two bytes containing the length
    cons.skip(2);

    for (r, g, b) in cons.pop_iter().take(len).tuples() {
        ws2812.write(r, g, b);
    }
    // wait for the state machine to write all bytes
    ws2812.flush();
    // let the neopixels latch on
    Timer::after_micros(60).await;
}

#[cfg(feature = "startup_fill")]
pub async fn fill_lights<P, const S: usize>((r, g, b): (u8, u8, u8), len: usize, ws2812: &mut Ws2812<'_, P, S>)
where
    P: Instance,
{
    for _ in 0..len {
        ws2812.write(r, g, b);
    }
    // wait for the state machine to write all bytes
    ws2812.flush();
    // let the neopixels latch on
    Timer::after_micros(60).await;
}
