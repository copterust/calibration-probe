#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::Pin;
use {defmt_rtt as _, panic_probe as _};

mod led_state;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    spawner.spawn(led_state::task(p.PB3.degrade())).unwrap();
    info!("Calibration Probe");
    led_state::idle();
}
