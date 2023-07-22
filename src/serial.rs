use embassy_stm32::usart::Uart;

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

    let mut msg: [u8; 1] = [0; 1];
    loop {
        usart.read(&mut msg).await.unwrap();
        usart.write(&msg).await.unwrap();
    }
}
