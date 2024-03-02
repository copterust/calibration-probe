use embassy_stm32::usart::Uart;
use defmt::*;

use crate::{mpu9250::FifoPacket, FIFO_BUF_SIZE, MPU_PIPE};

#[embassy_executor::task]
pub async fn echo_task(
    mut usart: Uart<
        'static,
        embassy_stm32::peripherals::USART2,
        embassy_stm32::peripherals::DMA1_CH7,
        embassy_stm32::peripherals::DMA1_CH6,
    >,
) {
    usart.write(b"\r\nCalibration Probe\r\n").await.unwrap();

    let mut msg: [u8; FIFO_BUF_SIZE] = [0; FIFO_BUF_SIZE];
    loop {
        MPU_PIPE.read(&mut msg).await;
        usart.write(&msg).await.unwrap();
        /*
        let ret = usart.read(&mut msg).await;
        if let Ok(_some) = ret {
            usart.write(&msg).await.unwrap();
        } else {
            info!("SERIAL: {:?}", ret);
        }
        */
    }
}
