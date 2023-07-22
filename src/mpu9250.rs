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
    fn configure_accelerometer(&self) {}
    fn configure_gyroscope(&self) {}

    async fn bits_set_clear(&mut self, reg: Register, set: u8, clear: u8) {
        let current = self.read(reg).await;
        let next = (current & !clear) | set;
        if next != current {
            self.write(reg, next).await;
        }
    }

    async fn write_config(&mut self) {
        for (reg, set, clear) in CONFIG {
            self.bits_set_clear(reg, set, clear).await;
        }
    }

    async fn validate_config(&mut self) -> Result<(), ()> {
        for (reg, set, clear) in CONFIG {
            let value = self.read(reg).await;
            if set > 0 && ((value & set) != set) {
                error!(
                    "MPU9250: register {:?} value {:#04x} bits {:#04x} not set",
                    reg, value, set
                );
                return Err(());
            }

            if clear > 0 && ((value & clear) != 0) {
                error!(
                    "MPU9250: register {:?} value {:#04x} bits {:#04x} not cleared",
                    reg, value, set
                );
                return Err(());
            }
        }

        Ok(())
    }

    async fn fifo_read_count(&mut self) -> u16 {
        let mut fifo_count_buf = [0u8; 3];
        let send_buffer = [Register::FifoCountH.read(), 0, 0];
        self.spi
            .transfer(&mut fifo_count_buf, &send_buffer)
            .await
            .unwrap();
        return ((fifo_count_buf[1] as u16) << 8) | fifo_count_buf[2] as u16;
    }

    async fn reset_fifo(&mut self) {
        self.write(Register::FifoEn, 0).await;
        self.bits_set_clear(
            Register::UserCtrl,
            UserCtrlBit::FifoRst as u8,
            UserCtrlBit::FifoEn as u8,
        )
        .await;
        self.write_config().await;
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Format)]
enum Register {
    WhoAmI = 0x75,
    PwrMgmt1 = 0x6B,
    SignalPathReset = 0x68,
    UserCtrl = 0x6A,
    Config = 0x1A,
    GyroConfig = 0x1B,
    AccelConfig = 0x1C,
    AccelConfig2 = 0x1D,
    FifoEn = 0x23,
    I2cMstCtrl = 0x24,

    I2cSlv4Ctrl = 0x34,
    IntPinCfg = 0x37,
    IntEnable = 0x38,
    I2cMstDelayCtrl = 0x67,

    XaOffsetH = 0x77,
    XaOffsetL = 0x78,

    YaOffsetH = 0x7A,
    YaOffsetL = 0x7B,

    ZaOffsetH = 0x7D,
    ZaOffsetL = 0x7E,

    FifoCountH = 0x72,
}

#[repr(u8)]
enum ConfigBit {
    /// When set to ‘1’, when the fifo is full, additional writes will not be written to fifo. 
    /// When set to ‘0’, when the fifo is full, additional writes will be written to the fifo, 
    /// replacing the oldest data.
    FifoMode = BIT6,
    /// Gyro: 3600 Hz bandwidth, 0.17 ms delay, 8kHz Fs
    DlpfCfgBypassDlpf8kHz = 7,
}

#[repr(u8)]
enum GyroConfigBit {
    /// Gyro Full Scale Select: +2000 dps
    GyroFsSel2000Dps = BIT4 | BIT3,
    /// Disable DLPF.
    /// The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b00
    FChoiceBBypassDlpf = BIT1 | BIT0,
}

#[repr(u8)]
enum AccelConfigBit {
    /// Accel Full Scale Select: ±16g
    AccelFsSel16g = BIT4 | BIT3,
}

#[repr(u8)]
enum AccelConfig2Bit {
    /// Bypass DLPF for accelerometer.
    /// 1.13 kHz bandwidth, 0.75 ms delay, noise density 250 ug/rtHz, rate 4 kHz
    AccelFChoiceBBypassDlpf = BIT3,
}

#[repr(u8)]
enum FifoEnBit {
    /// 1 – Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate;
    /// If enabled, buffering of data occurs even if data path is in standby.
    /// 0 – function is disabled
    GyroXout = BIT6,
    GyroYout = BIT5,
    GyroZout = BIT4,
    /// 1 – write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, 
    /// ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at the sample rate;
    /// 0 – function is disabled
    Accel = BIT3,
}

#[repr(u8)]
enum I2cSlv4CtrlBit {
    /// When enabled via the I2C_MST_DELAY_CTRL, those slaves will only be 
    /// enabled every (1+I2C_MST_DLY) samples (as determined by the 
    /// SMPLRT_DIV and DLPF_CFG registers.
    I2cMstDly = BIT4 | BIT3 | BIT2 | BIT1 | BIT0,
}

#[repr(u8)]
enum I2cMstCtrlBit {
    /// This bit controls the I2C Master’s transition from one slave read to the next 
    /// slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.
    I2cMstPNsr = BIT4,
    /// 400 kHz i2c master clock speed
    I2cMstClk400kHz = 13,
}

#[repr(u8)]
enum IntPinCfgBit {
    /// 1 – The logic level for INT pin is active low
    /// 0 – The logic level for INT pin is active high
    Actl = BIT7,
}

#[repr(u8)]
enum IntEnableBit {
    /// 1 – Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.
    /// The timing of the interrupt can vary depending on the setting in register 36 
    /// I2C_MST_CTRL, bit [6] WAIT_FOR_ES.
    /// 0 – function is disabled.
    RawRdyEn = BIT0,
}

#[repr(u8)]
enum I2cMstDelayCtrlBit {
    /// When enabled, slaves 0-4 will only be accessed (1+I2C_MST_DLY) samples 
    /// as determined by SMPLRT_DIV and DLPF_CFG
    I2cSlvxDlyEn = BIT4 | BIT3 | BIT2 | BIT1 | BIT0,
}

const CONFIG: [(Register, u8, u8); 18] = [
    (
        Register::Config,
        ConfigBit::FifoMode as u8 | ConfigBit::DlpfCfgBypassDlpf8kHz as u8,
        0,
    ),
    (
        Register::GyroConfig,
        GyroConfigBit::GyroFsSel2000Dps as u8,
        GyroConfigBit::FChoiceBBypassDlpf as u8,
    ),
    (
        Register::AccelConfig,
        AccelConfigBit::AccelFsSel16g as u8,
        0,
    ),
    (
        Register::AccelConfig2,
        AccelConfig2Bit::AccelFChoiceBBypassDlpf as u8,
        0,
    ),
    (
        Register::FifoEn,
        FifoEnBit::Accel as u8
            | FifoEnBit::GyroXout as u8
            | FifoEnBit::GyroYout as u8
            | FifoEnBit::GyroZout as u8,
        0,
    ),
    (Register::I2cSlv4Ctrl, I2cSlv4CtrlBit::I2cMstDly as u8, 0),
    (
        Register::I2cMstCtrl,
        I2cMstCtrlBit::I2cMstPNsr as u8 | I2cMstCtrlBit::I2cMstClk400kHz as u8,
        0,
    ),
    (Register::IntPinCfg, IntPinCfgBit::Actl as u8, 0),
    (Register::IntEnable, IntEnableBit::RawRdyEn as u8, 0),
    (
        Register::I2cMstDelayCtrl,
        I2cMstDelayCtrlBit::I2cSlvxDlyEn as u8,
        0,
    ),
    (
        Register::UserCtrl,
        UserCtrlBit::FifoEn as u8 | UserCtrlBit::I2cMstEn as u8 | UserCtrlBit::I2cIfDis as u8,
        0,
    ),
    (
        Register::PwrMgmt1,
        PwrMgmt1Bit::ClkSel0 as u8,
        PwrMgmt1Bit::Sleep as u8,
    ),
    (Register::XaOffsetH, 0, 0),
    (Register::XaOffsetL, 0, 0),
    (Register::YaOffsetH, 0, 0),
    (Register::YaOffsetL, 0, 0),
    (Register::ZaOffsetH, 0, 0),
    (Register::ZaOffsetL, 0, 0),
];

const BIT0: u8 = 1 << 0;
const BIT1: u8 = 1 << 1;
const BIT2: u8 = 1 << 2;
const BIT3: u8 = 1 << 3;
const BIT4: u8 = 1 << 4;
const BIT5: u8 = 1 << 5;
const BIT6: u8 = 1 << 6;
const BIT7: u8 = 1 << 7;

#[allow(unused)]
#[repr(u8)]
enum PwrMgmt1Bit {
    /// 1 – Reset the internal registers and restores the default settings. Write a 1 to 
    /// set the reset, the bit will auto clear.
    HReset = BIT7,
    /// When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit
    /// will be written here
    Sleep = BIT6,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    ClkSel0 = BIT0,
}

#[allow(unused)]
#[repr(u8)]
enum UserCtrlBit {
    /// 1 – Enable FIFO operation mode. 
    /// 0 – Disable FIFO access from serial interface. To disable FIFO writes by dma,
    /// use FIFO_EN register. To disable possible FIFO writes from DMP, disable the DMP.
    FifoEn = BIT6,
    /// 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from
    /// pins SDA/SDI and SCL/ SCLK.
    /// 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically 
    /// driven by pins SDA/SDI and SCL/ SCLK.
    /// NOTE: DMP will run when enabled, even if all internal sensors are disabled, 
    /// except when the sample rate is set to 8Khz
    I2cMstEn = BIT5,
    /// 1 – Reset I2C Slave module and put the serial interface in SPI mode only. 
    /// This bit auto clears after one clock cycle
    I2cIfDis = BIT4,
    /// 1 – Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
    FifoRst = BIT2,
    /// 1 – Reset I2C Master module. Reset is asynchronous. This bit auto clears after one clock cycle.
    /// NOTE: This bit should only be set when the I2C master has hung. If this bit 
    /// is set during an active I2C master transaction, the I2C slave will hang, which 
    /// will require the host to reset the slave
    I2cMstRst = BIT1,
    /// 1 – Reset all gyro digital signal path, accel digital signal path, and temp 
    /// digital signal path. This bit also clears all the sensor registers. 
    /// SIG_COND_RST is a pulse of one clk8M wide
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
                    info!("MPU9250: out of reset");
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
            State::Configure => {
                mpu.write_config().await;
                if mpu.validate_config().await.is_err() {
                    info!("MPU9250: configuration failed, resetting");
                    Timer::after(Duration::from_micros(1_000_000)).await;
                    mpu.state = State::Reset;
                    continue;
                } else {
                    info!("MPU9250: configured, running...");
                }
                mpu.configure_accelerometer();
                mpu.configure_gyroscope();
                // TODO init mag
                mpu.state = State::FifoRead;
                // TODO drdy
                mpu.reset_fifo().await;
            }
            State::FifoRead => {
                let fifo_count = mpu.fifo_read_count().await;
                info!("Fifo count {}", fifo_count);
                Timer::after(Duration::from_micros(100_000)).await;
            }
        }
    }
}

enum State {
    Reset,
    AfterReset,
    Configure,
    FifoRead,
}
