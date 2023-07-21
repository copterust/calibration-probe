#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_stm32::gpio::Pin;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use {defmt_rtt as _, panic_probe as _};

mod led_state;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(led_state::task(p.PB3.degrade())).unwrap();
    led_state::idle();

    let mut usart_config = Config::default();
    usart_config.baudrate = 460800;

    let mut usart = Uart::new(
        p.USART2,
        p.PA15,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        usart_config,
    );

    usart.write(b"\r\nCalibration Probe\r\n").await.unwrap();

    let mut msg: [u8; 1] = [0; 1];
    loop {
        usart.read(&mut msg).await.unwrap();
        usart.write(&msg).await.unwrap();
    }
}

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});
