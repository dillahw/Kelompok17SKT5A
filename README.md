
Main.rs :
#![no_std]
#![cfg_attr(not(test), no_main)]

use panic_halt as _;
use esp_hal::{
    Config,
    uart::{Uart, Config as UartConfig},
    gpio::{Output, Level, OutputConfig},
    time::{Instant, Duration},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const BAUD: u32 = 9_600;
const SID:  u8  = 1;
const TURNAROUND_SPINS: u32 = 50_000;
const TIMEOUT_SPINS:    u32 = 200_000;
const SETPOINT_TEMP: f32 = 33.0; // suhu target ON kipas

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(Config::default());

    // UART1: TX=GPIO18, RX=GPIO17
    let mut uart = Uart::new(p.UART1, UartConfig::default().with_baudrate(BAUD))
        .expect("UART1 init failed")
        .with_tx(p.GPIO18)
        .with_rx(p.GPIO17);

    // Inisialisasi L9110 (INA=GPIO10, INB=GPIO7)
    let mut fan_ina = Output::new(p.GPIO10, Level::Low, OutputConfig::default());
    let mut fan_inb = Output::new(p.GPIO7, Level::Low, OutputConfig::default());
    println!("L9110 initialized (INA=GPIO10, INB=GPIO7) -> OFF");

    loop {
        // ====== Baca Suhu ======
        let mut req = [0u8; 8];
        req[0] = SID;
        req[1] = 0x04;
        req[2..4].copy_from_slice(&0x0001u16.to_be_bytes());
        req[4..6].copy_from_slice(&1u16.to_be_bytes());
        let crc1 = crc16(&req[..6]);
        req[6] = (crc1 & 0xFF) as u8;
        req[7] = (crc1 >> 8) as u8;

        let _ = uart.write(&req);
        let _ = uart.flush();
        short_spin(TURNAROUND_SPINS);

        let mut rx = [0u8; 32];
        let mut n = 0usize;
        let mut spins = 0u32;
        let mut t: Option<f32> = None;

        while spins < TIMEOUT_SPINS && n < rx.len() {
            let mut b = [0u8; 1];
            match uart.read(&mut b) {
                Ok(1) => { rx[n] = b[0]; n += 1; if n >= 7 { break; } }
                _ => { short_spin(1_000); spins += 1; }
            }
        }

        if n >= 7 && (rx[1] & 0x80) == 0 && rx[2] == 2 && check_crc(&rx[..n]) {
            let raw_t = u16::from_be_bytes([rx[3], rx[4]]);
            t = Some(raw_t as f32 / 10.0);
        } else {
            println!("RX(t): {:?}", &rx[..n]);
        }

        // ====== Kontrol Kipas ======
        if let Some(temp) = t {
            if temp >= SETPOINT_TEMP {
                println!("Temp {:.1}°C >= {:.1}°C -> menyalakan kipas setelah 2s delay", temp, SETPOINT_TEMP);
                sleep(Duration::from_millis(2000));
                fan_ina.set_high();
                fan_inb.set_low();
                println!("FAN: ON (INA=HIGH, INB=LOW)");
            } else {
                fan_ina.set_low();
                fan_inb.set_low();
                println!("FAN: OFF (temp {:.1}°C)", temp);
            }
        } else {
            fan_ina.set_low();
            fan_inb.set_low();
            println!("FAN: OFF (no temp)");
        }

        sleep(Duration::from_millis(200));

        // ====== Baca Kelembaban ======
        req[2..4].copy_from_slice(&0x0002u16.to_be_bytes());
        let crc2 = crc16(&req[..6]);
        req[6] = (crc2 & 0xFF) as u8;
        req[7] = (crc2 >> 8) as u8;

        let _ = uart.write(&req);
        let _ = uart.flush();
        short_spin(TURNAROUND_SPINS);

        rx.fill(0);
        n = 0;
        spins = 0;
        let mut rh: Option<f32> = None;

        while spins < TIMEOUT_SPINS && n < rx.len() {
            let mut b = [0u8; 1];
            match uart.read(&mut b) {
                Ok(1) => { rx[n] = b[0]; n += 1; if n >= 7 { break; } }
                _ => { short_spin(1_000); spins += 1; }
            }
        }

        if n >= 7 && (rx[1] & 0x80) == 0 && rx[2] == 2 && check_crc(&rx[..n]) {
            let raw_rh = u16::from_be_bytes([rx[3], rx[4]]);
            rh = Some(raw_rh as f32 / 10.0);
        } else {
            println!("RX(rh): {:?}", &rx[..n]);
        }

        // ====== Kirim JSON (Prefix [DATA]) ======
        match (rh, t) {
            (Some(rh_val), Some(t_val)) => {
                println!(r#"[DATA]{{"rh":{:.1},"t":{:.1}}}"#, rh_val, t_val);
            }
            (Some(rh_val), None) => {
                println!(r#"[DATA]{{"rh":{:.1},"t":null}}"#, rh_val);
            }
            (None, Some(t_val)) => {
                println!(r#"[DATA]{{"rh":null,"t":{:.1}}}"#, t_val);
            }
            _ => {
                println!(r#"[DATA]{{"rh":null,"t":null}}"#);
            }
        }

        sleep(Duration::from_millis(1000));
    }
}

// ===== Utils =====
#[inline(always)]
fn sleep(dur: Duration) {
    let start = Instant::now();
    while start.elapsed() < dur {
        core::hint::spin_loop();
    }
}

fn short_spin(iter: u32) { for _ in 0..iter { core::hint::spin_loop(); } }

fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if (crc & 1) != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    crc
}

fn check_crc(frame: &[u8]) -> bool {
    if frame.len() < 3 { return false; }
    let calc = crc16(&frame[..frame.len() - 2]);
    frame[frame.len() - 2] == (calc & 0xFF) as u8 && frame[frame.len() - 1] == (calc >> 8) as u8
}



