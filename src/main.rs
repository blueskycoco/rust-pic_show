#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, BufferedUart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_stm32::exti::{ExtiInput, AnyChannel, Channel};
use embassy_stm32::gpio::{AnyPin, Level, Output, Input, Pull, Pin, Speed};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[embassy_executor::task]
async fn blinky(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High, Speed::Low);

    loop {
        //info!("high");
        led.set_high();
        Timer::after_millis(300).await;

        //info!("low");
        led.set_low();
        Timer::after_millis(300).await;
    }
}

#[embassy_executor::task]
async fn btn(pin: AnyPin, ch: AnyChannel) {
    let mut button = ExtiInput::new(pin, ch, Pull::Up);

    info!("Press the USER button...");

    loop {
        button.wait_for_falling_edge().await;
        info!("Pressed!");
        button.wait_for_rising_edge().await;
        info!("Released!");
    }
}

/*
fn clear(ary: &mut [u8]) {
    ary.iter_mut().for_each(|m| *m = 0)
}

async fn usr_cmd(rx: &mut BufferedUartRx<'_>,
                    tx: &mut BufferedUartTx<'_>,
                    cmd: &str,
                    s: &mut [u8]) {
    clear(s);
    unwrap!(tx.write_all(cmd.as_bytes()).await);
    let mut cnt = 0;
    loop {
        let n = rx.read(&mut s[cnt..]).await;
        match n {
            Ok(bytes) => cnt = cnt + bytes,
            Err(e) => info!("read error {}", e),
        }

        if s.get(cnt - 4) == Some(&b'\r') && s.get(cnt - 3) == Some(&b'\n') &&
           s.get(cnt - 2) == Some(&b'\r') && s.get(cnt - 1) == Some(&b'\n') {
            let str_resp = core::str::from_utf8(s).unwrap();
            info!("{}", str_resp);
            break;
        }
    }
}
*/
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");
    spawner.spawn(blinky(p.PA5.degrade())).unwrap();
    spawner.spawn(btn(p.PC13.degrade(), p.EXTI13.degrade())).unwrap();
    
    static TX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 128])[..];
    static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 128])[..];
    
    let config = Config::default();
    let usart = BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf,
                                      rx_buf, config).unwrap();
    let (mut usr_tx, mut usr_rx) = usart.split();
    let mut pic  = [0u8; 2048];
    loop {
        unwrap!(usr_tx.write_all("send ok".as_bytes()).await);
        unwrap!(usr_rx.read_exact(&mut pic).await);
    }
}
