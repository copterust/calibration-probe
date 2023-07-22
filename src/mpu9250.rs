use defmt::*;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    peripherals::{DMA1_CH2, DMA1_CH3, SPI1},
    spi::Spi,
};
use embassy_time::{Duration, Timer};

pub struct Mpu9250 {
    spi: Spi<'static, SPI1, DMA1_CH3, DMA1_CH2>,
    ncs: Output<'static, AnyPin>,
}

impl Mpu9250 {
    pub async fn probe(&mut self) -> Result<(), ()> {
        let who_am_i = self.read(Register::WhoAmI).await;
        if WHO_AM_I == who_am_i {
            Ok(())
        } else {
            Err(())
        }
    }

    async fn read(&mut self, reg: Register) -> u8 {
        let mut buffer = [0x0; 2];
        info!("{}", WHO_AM_I);
        for i in 0..255 {
            let send_buffer = [(i as u8), 0x0];

            self.ncs.set_low();
            self.spi.transfer(&mut buffer, &send_buffer).await.unwrap();
            info!("{}: received {:?}", i, buffer);
            self.ncs.set_high();
            Timer::after(Duration::from_millis(5)).await;
        }
        buffer[1]
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Register {
    WhoAmI = 0x75,
}

const WHO_AM_I: u8 = 0x71;
const W: u8 = 0 << 7;

impl Register {
    fn read(&self) -> u8 {
        *self as u8 | 0x80
    }

    fn write(&self) -> u8 {
        *self as u8 | W
    }
}

pub fn new(spi: Spi<'static, SPI1, DMA1_CH3, DMA1_CH2>, ncs: AnyPin) -> Mpu9250 {
    let ncs = Output::new(ncs, Level::High, Speed::Low);

    Mpu9250 { spi: spi, ncs: ncs }
}

#[embassy_executor::task]
async fn spi_task() {}
