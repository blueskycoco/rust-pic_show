#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_stm32::gpio::{Level, Output, Speed, Pin, AnyPin};
use embassy_time::Timer;
use embassy_stm32::time::Hertz;
use embedded_io_async::BufRead;
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;
use md5_rs::Context;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

fn clear(ary: &mut [u8]) {
    ary.iter_mut().for_each(|m| *m = 0)
}

#[embassy_executor::task]
async fn blinky(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after_millis(300).await;

        led.set_low();
        Timer::after_millis(300).await;
    }
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
/*
async fn usr_init(usart: &mut Uart<'_, embassy_stm32::mode::Async>) -> bool {
    let mut s = [0u8; 128];
    unwrap!(usart.write("+++".as_bytes()).await);
    unwrap!(usart.read_until_idle(&mut s).await);
    unwrap!(usart.write("a".as_bytes()).await);
    unwrap!(usart.read_until_idle(&mut s).await);

    Timer::after_millis(300).await;
    usr_cmd(usart, "at+wskey=wpa2psk,aes,DUBB-JcJf-kU4g-C3IY\r", &mut s).await;
    usr_cmd(usart, "at+wsssid=8848\r", &mut s).await;
    usr_cmd(usart, "at+wmode=sta\r", &mut s).await;
    loop {
        unwrap!(usart.write("at+ping=172.20.10.6\r".as_bytes()).await);
        loop {
            unwrap!(usart.read_until_idle(&mut s).await);
            let str_resp = core::str::from_utf8(&s).unwrap();
            info!("{}", str_resp);
            if str_resp.contains("Success") {
                usr_cmd(usart, "at+wann\r", &mut s).await;
                usr_cmd(usart, "at+netp=tcp,server,1234,172.20.10.8\r", &mut s)
                        .await;
                usr_cmd(usart, "at+netp\r", &mut s).await;
                //usr_cmd(usart, "at+tcpdis=on\r").await;
                usr_cmd(usart, "at+tcpdis\r", &mut s).await;
                Timer::after_millis(100).await;
                return true;
            } else if str_resp.contains("+ok") || str_resp.contains("+ERR") {
                break;
            }
            clear(&mut s);
        }
        Timer::after_millis(1000).await;
    }
}
*/
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // Oscillator for bluepill, Bypass for nucleos.
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);
    //let p = embassy_stm32::init(Default::default());

    spawner.spawn(blinky(p.PB0.degrade())).unwrap();
    let config = Config::default();
    static TX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 128])[..];
    static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 128])[..];
    let usart = BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf,
                                      rx_buf, config).unwrap();
    let (mut usr_tx, mut usr_rx) = usart.split();
    //let mut usart = Uart::new(p.USART2, p.PA3, p.PA2, Irqs, p.DMA1_CH7,
    //p.DMA1_CH6, config).unwrap();
    let mut rst = Output::new(p.PA0, Level::High, Speed::Low);

    // reset usr_wifi232_t
    Timer::after_millis(200).await;
    rst.set_low();
    Timer::after_millis(300).await;
    rst.set_high();
    Timer::after_millis(1500).await;

    // enter at command mode
    let mut s = [0u8; 128];
    unwrap!(usr_tx.write_all("+++".as_bytes()).await);
    unwrap!(usr_rx.read(&mut s).await);
    unwrap!(usr_tx.write_all("a".as_bytes()).await);
    unwrap!(usr_rx.read(&mut s).await);
    // waiting finish
    Timer::after_millis(500).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+wmode=sta\r", &mut s).await;
    //usr_cmd(&mut usart, "at+netp=TCP,Server,1234,172.20.10.2\r", &mut s).await;
    usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcpdis=on\r", &mut s).await;

    loop {
        let mut ss = [0u8; 128];
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+wann\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+netp\r", &mut s).await;
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+tcplk\r", &mut s).await;
        let tcplk = core::str::from_utf8(&s).unwrap();
        usr_cmd(&mut usr_rx, &mut usr_tx, "at+ping=172.20.10.13\r", &mut ss).await;
        let ping = core::str::from_utf8(&ss).unwrap();
        if ping.contains("Success") && tcplk.contains("on") {
            info!("network stable!");
            usr_cmd(&mut usr_rx, &mut usr_tx, "at+entm\r", &mut s).await;
            break;
        }
        Timer::after_millis(2000).await;
    }
    let mut pic  = [0u8; 7840];
    loop {
        unwrap!(usr_tx.write_all("send ok".as_bytes()).await);
        unwrap!(usr_rx.read_exact(&mut pic).await);
        let mut ctx = Context::new();
        ctx.read(&pic[22..]);
        let digest = ctx.finish();
        let remote_dig = &pic[2..18];
        if digest != remote_dig {
            error!("md5 missmatch");
            error!("L {:?}", digest);
            error!("R {:?}", remote_dig);
        } else {
            info!("pic send ok");
        }
    }
}
