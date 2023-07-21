How to read magnetic, angular rate, and gravity readings from sensor and send them over UART.

We will use the following components for this project:
* MPU9250 sensor board
* NUCLEO-F303K8

Let's setup cargo runner first. This would give us nice way to debug our code later. This is done in `.cargo/config.toml`:

```toml
runner = "probe-run --chip STM32F303K8Tx --connect-under-reset"
```

Note use of `--connect-under-reset` here which is needed for some chips.

Configure toolchain in `rust-toolchain.toml`: use nightly for embassy and set Cortex-M4 target with floating point support.

```toml
[toolchain]
channel = "nightly"
components = [ "rust-src", "rustfmt" ]

targets = [
    "thumbv7em-none-eabihf",
]
```

Let's check that everything works:

```rust
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = embassy_stm32::init(Default::default());
    info!("Calibration Probe");
}
```

Run it with `cargo run --release` and you should see output right in the console:
```
0.000030 INFO  Calibration Probe
```

Time to blink LED! For that we need to import additional items.
```rust
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
```

Our LED would reflect several states of the firmware:
* Off: default, firmware is not ready
* Idle: firmware initialized, awaiting for commands
* Error: firmware failed to init mpu
* Active: firmware is sending something to host

```rust
#[atomic_enum]
pub enum LedState {
    Off = 0,
    Idle = 10,
    Error = 20,
    BlinkOnce = 30,
}
```

To pass this enum around we will use `atomic_enum` crate (first place to optimize)
and save it in global variable. AtomicLedState was auto-created from LedState.

```rust
static LED_STATE: AtomicLedState = AtomicLedState::new(LedState::Off);
```

To declare a task you use `#[embassy_executor::task]` followed by async function.

```rust
#[embassy_executor::task]
async fn led_status_task(led: AnyPin)
```

We could pass any pin to our task later. Setting this pin as output is easy.
```rust
let mut led = Output::new(led, Level::Low, Speed::Low);
```

Now we ready to enter main led blinking loop where we read required blinking state and execute it.

```rust
loop {
    let state = LED_STATE.load(Ordering::Relaxed);
    match state {
    ...
```

Spawning this task is also easy.

```rust
    let p = embassy_stm32::init(Default::default());
    spawner.spawn(led_status_task(p.PB3.degrade())).unwrap();
```