# pico-w-neopixel-server

# **!WIP!** Rust migration in progress!

To flash it to pico just put the pico in bootsel mode and run
```sh
cargo run --release
```

> [!TIP] 
> Some lights have their red and green colors swapped, if you don't want to compensate for that in each frame sent to pico, you can compensate in the firmware:
```sh
cargo run --release --features grb
```


Simple webserver aiming to be a thin remote controller for neopixel lights.

Developed to work with https://github.com/mrozycki/rustmas project.

The C++ SDK version using Pico SDK can be found here: https://github.com/krzmaz/pico-w-neopixel-server-cpp
