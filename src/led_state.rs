#![allow(unused)]
use atomic_enum::atomic_enum;
use core::sync::atomic::Ordering;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_time::{Duration, Timer};

static LED_STATE: AtomicLedState = AtomicLedState::new(LedState::Off);

pub fn off() {
    LED_STATE.store(LedState::Off, Ordering::Relaxed);
}

pub fn idle() {
    LED_STATE.store(LedState::Idle, Ordering::Relaxed);
}

pub fn error() {
    LED_STATE.store(LedState::Error, Ordering::Relaxed);
}

pub async fn blink() {
    let state: LedState = LED_STATE.load(Ordering::Relaxed);
    LED_STATE.store(LedState::Error, Ordering::Relaxed);
    Timer::after(Duration::from_millis(2000)).await; // long enough for every state
    LED_STATE.store(state, Ordering::Relaxed);
}

#[atomic_enum]
enum LedState {
    Off = 0,
    Idle = 10,
    Error = 20,
    BlinkOnce = 30,
}

impl Default for LedState {
    fn default() -> Self {
        Self::Off
    }
}

#[embassy_executor::task]
pub async fn task(led: AnyPin) {
    let mut led = Output::new(led, Level::Low, Speed::Low);

    loop {
        let state: LedState = LED_STATE.load(Ordering::Relaxed);
        match state {
            LedState::Off => {
                led.set_low();
                Timer::after(Duration::from_millis(5)).await;
            }
            LedState::Idle => {
                led.set_high();
                Timer::after(Duration::from_millis(5)).await;
                led.set_low();
                Timer::after(Duration::from_millis(1970)).await;
            }
            LedState::Error => {
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
                led.set_low();
                Timer::after(Duration::from_millis(500)).await;
            }
            LedState::BlinkOnce => {
                led.set_low();
                Timer::after(Duration::from_millis(10)).await;
                led.set_high();
                Timer::after(Duration::from_millis(5)).await;
                led.set_low();
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}
