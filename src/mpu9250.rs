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
    state: State,
}

impl Mpu9250 {
    pub async fn reset(&mut self) {
        self.write(Register::PwrMgmt1, PwrMgmt1Bit::HReset as u8)
            .await;
        Timer::after(Duration::from_micros(100_000)).await;
    }

    async fn write(&mut self, reg: Register, value: u8) {
        let send_buffer = [reg.write(), value];
        let mut buffer: [u8; 2] = [0x0; 2];

        self.ncs.set_low();
        self.spi.transfer(&mut buffer, &send_buffer).await.unwrap();
        self.ncs.set_high();
    }

    pub async fn probe(&mut self) -> Result<(), u8> {
        let whoami = self.read(Register::WhoAmI).await;

        if WHO_AM_I != whoami {
            info!("MPU9250: unexpected WHO_AM_I {:#04x}", whoami);
            return Err(whoami);
        }

        Ok(())
    }

    async fn read(&mut self, reg: Register) -> u8 {
        let mut buffer: [u8; 2] = [0x0; 2];
        let send_buffer = [reg.read(), 0x0];
        self.ncs.set_low();
        self.spi.transfer(&mut buffer, &send_buffer).await.unwrap();
        self.ncs.set_high();
        buffer[1]
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Register {
    WhoAmI = 0x75,
    PwrMgmt1 = 0x6B,
    SignalPathReset = 0x68,
    UserCtrl = 0x6A,
}

const BIT0: u8 = 1 << 0;
const BIT1: u8 = 1 << 1;
const BIT2: u8 = 1 << 2;
const BIT4: u8 = 1 << 4;
const BIT5: u8 = 1 << 5;
const BIT6: u8 = 1 << 6;
const BIT7: u8 = 1 << 7;

#[allow(unused)]
#[repr(u8)]
enum PwrMgmt1Bit {
    HReset = BIT7,
    Sleep = BIT6,
    ClkSel0 = BIT0,
}

#[allow(unused)]
#[repr(u8)]
enum UserCtrlBit {
    FifoEn = BIT6,
    I2cMstEn = BIT5,
    I2cIfDis = BIT4,
    FifoRst = BIT2,
    I2cMstRst = BIT1,
    SigCondRst = BIT0,
}

#[repr(u8)]
enum SignalPathResetBit {
    GyroReset = BIT2,
    AccelReset = BIT1,
    TempReset = BIT0,
}

const WHO_AM_I: u8 = 0x73;
const W: u8 = 0 << 7;

impl Register {
    fn read(&self) -> u8 {
        *self as u8 | BIT7
    }

    fn write(&self) -> u8 {
        *self as u8 | W
    }
}

pub fn new(spi: Spi<'static, SPI1, DMA1_CH3, DMA1_CH2>, ncs: AnyPin) -> Mpu9250 {
    let ncs = Output::new(ncs, Level::High, Speed::Low);

    Mpu9250 {
        spi,
        ncs,
        state: State::Reset,
    }
}

#[embassy_executor::task]
pub async fn task(mut mpu: Mpu9250) {
    info!("MPU9250: task started");
    loop {
        match mpu.state {
            State::Reset => {
                info!("MPU9250: resetting...");
                mpu.reset().await;
                mpu.state = State::AfterReset;
                continue;
            }
            State::AfterReset => {
                if mpu.probe().await.is_ok() && 0x01 == mpu.read(Register::PwrMgmt1).await {
                    info!("MPU9250: reset done");
                    mpu.write(Register::PwrMgmt1, PwrMgmt1Bit::ClkSel0 as u8)
                        .await;
                    mpu.write(
                        Register::SignalPathReset,
                        SignalPathResetBit::GyroReset as u8
                            | SignalPathResetBit::AccelReset as u8
                            | SignalPathResetBit::TempReset as u8,
                    )
                    .await;
                    mpu.write(
                        Register::UserCtrl,
                        UserCtrlBit::I2cMstEn as u8
                            | UserCtrlBit::SigCondRst as u8
                            | UserCtrlBit::I2cIfDis as u8
                            | UserCtrlBit::I2cMstRst as u8,
                    )
                    .await;
                    Timer::after(Duration::from_micros(100_000)).await;
                    mpu.state = State::Configure;
                } else {
                    info!("MPU9250: reset failed, retrying");
                    Timer::after(Duration::from_micros(1_000_000)).await;
                    mpu.state = State::Reset;
                }
                continue;
            }
            State::Configure => {}
        }
    }
}

enum State {
    Reset,
    AfterReset,
    Configure,
}
