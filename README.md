# pico-w-neopixel-server

Simple webserver aiming to be a thin remote controller for neopixel lights.

Developed to work with https://github.com/mrozycki/rustmas project.

## Setup
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


---
The legacy version of this project using Pico C++ SDK can be found here: https://github.com/krzmaz/pico-w-neopixel-server-cpp
