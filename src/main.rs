#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_stm32::{gpio::{Pin, AnyPin, Input, Pull}, exti::{ExtiInput, AnyChannel, Channel}, Peripherals};
use {defmt_rtt as _, panic_probe as _};
use defmt::*;

mod led_state;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(led_state::task(p.PB3.degrade())).unwrap();
    led_state::idle();

    spawner.spawn(data_ready(p.PA11.degrade(), p.EXTI13.degrade())).unwrap();
}

#[embassy_executor::task]
async fn data_ready(pin: AnyPin, ch: AnyChannel) {
    let drdy_input = Input::new(pin, Pull::None);
    let mut drdy = ExtiInput::new(drdy_input, ch);
    loop {
        drdy.wait_for_rising_edge().await;
        info!("Interrupt");
    }
}

