#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::Pin;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{exti::Channel, time::Hertz};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

mod led_state;
mod mpu9250;
mod serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(led_state::task(p.PB3.degrade())).unwrap();
    led_state::idle();

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

    let spi = Spi::new(
        p.SPI1,
        p.PA5, // A4  SCL
        p.PB5, // D11 SDA
        p.PB4, // D12 AD0
        p.DMA1_CH3,
        p.DMA1_CH2,
        Hertz(1_000_000),
        SpiConfig::default(),
    );

    let mpu: mpu9250::Mpu9250 =
        mpu9250::new(spi, p.PA0.degrade(), p.PA9.degrade(), p.EXTI13.degrade()); // D1
    spawner.spawn(mpu9250::task(mpu)).unwrap();

    loop {
        Timer::after(Duration::from_micros(1_000_000)).await;
    }
}

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});
