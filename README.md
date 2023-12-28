# pico-w-neopixel-server

Simple webserver aiming to be a thin remote controller for neopixel lights.

Developed to work with https://github.com/mrozycki/rustmas project.

## Setup
### Wiring
See [wiring.md](docs/wiring.md)
### Dependencies
You will need to use [picotool](https://github.com/raspberrypi/picotool) or [elf2uf2-rs](https://github.com/JoNil/elf2uf2-rs) to flash the compiled binary onto your pico.  
Choose whichever you want and set the runner in `.cargo/config.toml` accordingly

We recommend using `picotool`, as it let's you read the binary information from a flashed pico as well.

On MacOS and Linux you can install it with [brew](https://brew.sh/):

```sh
brew install picotool
```
### WiFi credentials
You will need to make a copy of the `src/secret.rs.example` file and name it `src/secret.rs`.  
In that file fill you the SSID and password of the WiFi network you want the pico to connect to.


## Flashing

This repo contains both TCP and UDP versions of the neopixel server. The TCP one is used by default.


To flash it to pico just put the pico in bootsel mode and run
```sh
cargo run --release
```

> [!NOTE] 
> Some lights have their red and green colors swapped, if you don't want to compensate for that in each frame sent to pico, you can compensate in the firmware:
```sh
cargo run --release --features grb
```
### UDP
To use the UDP version you just need to add `--bin udp` to your flash command:
```sh
cargo run --release --bin udp
```
> [!IMPORTANT]  
> Make sure that the UDP packets length doesn't exceed the MTU in your network.
> For a default MTU of 1500 bytes, you won't be able to drive 500 LEDs due to the protocol overhead.

## Testing
The protocol is as follows: the length of the data is send on the first two bytes (Little Endian) and then all the colors are sent in RGB order. For example this data:
```
03 00 7F 00 00
```
will turn on the first LED to red with half intensity. On linux it can be tested quickly by doing:
```
printf '\x03\x00\x7F\x00\x00' | nc -v <IP_ADDRESS> 1234 
```

If you build the project in debug mode (omit the `--release` in `cargo run`), you should also see the received bytes in the logs.

---
The legacy version of this project using Pico C++ SDK can be found here: https://github.com/krzmaz/pico-w-neopixel-server-cpp
