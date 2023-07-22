#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{
    exti::{AnyChannel, Channel, ExtiInput},
    gpio::{AnyPin, Input, Pin, Pull},
    time::Hertz,
    Peripherals,
};
use {defmt_rtt as _, panic_probe as _};

mod led_state;
mod serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(led_state::task(p.PB3.degrade())).unwrap();
    led_state::idle();

    spawner
        .spawn(data_ready(p.PA11.degrade(), p.EXTI13.degrade()))
        .unwrap();

    let mut usart_config = UsartConfig::default();
    usart_config.baudrate = 460800;

    let usart = Uart::new(
        p.USART2,
        p.PA15,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        usart_config,
    );

    spawner.spawn(serial::echo_task(usart)).unwrap();
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

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
