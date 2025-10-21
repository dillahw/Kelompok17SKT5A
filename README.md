Cargo.toml :
[package]
edition      = "2021"
name         = "hello-rust"
rust-version = "1.86"
version      = "0.1.0"

[[bin]]
name = "hello-rust"
path = "./src/bin/main.rs"
test = false
doctest = false
bench = false

[dependencies]
esp-bootloader-esp-idf  = { version = "0.2.0", features = ["esp32s3"] }
esp-hal                = { version = "=1.0.0-rc.0", features = ["esp32s3"] }
esp-println             = { version = "0.10", features = ["esp32s3"] }
panic-halt = "0.2"
fugit = "0.3"


critical-section = "1.2.0"


[profile.dev]
# Rust debug is too slow.
# For debug builds always builds with some optimization
opt-level = "s"

[profile.release]
codegen-units    = 1     # LLVM can perform better optimizations using a single thread
debug            = 2
debug-assertions = false
incremental      = false
lto              = 'fat'
opt-level        = 's'
overflow-checks  = false

[lib]
crate-type = ["rlib"]
test = false
doctest = false
bench = false

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

gateway.py :
import serial
import json
import time
from datetime import datetime
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WritePrecision, WriteOptions

# === KONFIGURASI SERIAL ===
SERIAL_PORT = "COM4"          # sesuaikan dengan port ESP32 kamu
BAUDRATE = 115200

# === KONFIGURASI INFLUXDB ===
INFLUX_URL = "http://127.0.0.1:8086"
INFLUX_TOKEN = "9rymG95d933BZ8ZrrXmx4xRTGUbn8u-lxypawC4VDCpNcHnYWbY6f3TDYwPnQjy-Wv60cla7FZEJ5YvijlLiKg=="
INFLUX_ORG = "d92b85ad6e4f67f0"
INFLUX_BUCKET = "skt12"

# === KONFIGURASI THINGSBOARD ===
THINGSBOARD_HOST = "demo.thingsboard.io"
ACCESS_TOKEN = "s4NVxCgdexJLVTvw4eTp"

# === SETUP SERIAL ===
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
    print(f"✅ Serial {SERIAL_PORT} terbuka ({BAUDRATE} baud)")
except Exception as e:
    print(f"❌ Gagal membuka port serial {SERIAL_PORT}: {e}")
    exit()

# === SETUP MQTT (THINGSBOARD) ===
mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
mqtt_client.username_pw_set(ACCESS_TOKEN)
try:
    mqtt_client.connect(THINGSBOARD_HOST, 1883, 60)
    print("✅ Tersambung ke ThingsBoard")
except Exception as e:
    print(f"⚠ Tidak bisa terhubung ke ThingsBoard: {e}")

# === SETUP INFLUXDB ===
try:
    influx = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = influx.write_api(write_options=WriteOptions(batch_size=1))
    print("✅ Koneksi ke InfluxDB berhasil")
except Exception as e:
    print(f"⚠ Tidak bisa konek ke InfluxDB: {e}")

print("\n🚀 Gateway berjalan — membaca data dari ESP32-S3 dan kirim ke InfluxDB + ThingsBoard\n")

# === LOOP PEMBACAAN SERIAL ===
while True:
    try:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue

        print(f"Log dari ESP32: {line}")
        try:
            data = json.loads(line)  # contoh: {"rh":55.3,"t":35.1}
        except json.JSONDecodeError:
            print(f"⚠ Data tidak valid dari serial: {line}")
            continue

        # === KIRIM KE THINGSBOARD ===
        try:
            mqtt_client.publish("v1/devices/me/telemetry", json.dumps(data))
            print("📤 Data terkirim ke ThingsBoard")
        except Exception as e:
            print("⚠ Gagal kirim ke ThingsBoard:", e)

        # === KIRIM KE INFLUXDB ===
        try:
            t = float(data.get("t", 0.0))
            rh = float(data.get("rh", 0.0))
            point = (
                Point("sensor_data")
                .tag("device", "ESP32S3")
                .field("temperature", t)
                .field("humidity", rh)
                .time(datetime.utcnow(), WritePrecision.NS)
            )
            print("📤 Menulis ke InfluxDB:", point.to_line_protocol())
            write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
            print(f"✅ Data tersimpan ke InfluxDB: Temp={t}°C  RH={rh}%\n")
        except Exception as e:
            print("❌ Error kirim ke InfluxDB:", e)

        time.sleep(1)

    except Exception as e:
        print("❌ Error umum:", e)
        time.sleep(2)

dwism.py :
import os
import time
import xml.etree.ElementTree as ET
import zipfile
import io
from datetime import datetime
from influxdb_client import InfluxDBClient, Point

# =====================================================
# KONFIGURASI INFLUXDB
# =====================================================
INFLUX_URL = "http://127.0.0.1:8086"
INFLUX_TOKEN = "9rymG95d933BZ8ZrrXmx4xRTGUbn8u-lxypawC4VDCpNcHnYWbY6f3TDYwPnQjy-Wv60cla7FZEJ5YvijlLiKg=="
INFLUX_ORG = "d92b85ad6e4f67f0"
INFLUX_BUCKET = "skt12"

# =====================================================
# KONFIGURASI FILE DWSIM
# =====================================================
DWSIM_FILE = r"C:\Users\ASUS\Downloads\RUST_DCS-main\DWISMKEL17.dwxmz"
LOG_FILE = r"C:\Users\ASUS\Downloads\hello-rust\scripts\dwsim_log.txt"
POLL_INTERVAL = 5  # detik antar pembacaan

# =====================================================
# FUNGSI LOGGING
# =====================================================
def log(message: str):
    timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
    msg = f"{timestamp} {message}"
    print(msg)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(msg + "\n")

# =====================================================
# FUNGSI PARSE FILE DWSIM
# =====================================================
def parse_dwsim_xml(file_path):
    if not os.path.exists(file_path):
        log(f"❌ File tidak ditemukan: {file_path}")
        return []

    try:
        with zipfile.ZipFile(file_path, 'r') as z:
            xml_name = None
            for name in z.namelist():
                if name.endswith(".xml"):
                    xml_name = name
                    break

            if not xml_name:
                log("❌ Tidak ditemukan file XML di dalam DWSIM ZIP.")
                return []

            with z.open(xml_name) as xml_file:
                tree = ET.parse(io.TextIOWrapper(xml_file, encoding='utf-8'))
                root = tree.getroot()

    except zipfile.BadZipFile:
        log("❌ File DWSIM bukan format ZIP yang valid.")
        return []
    except ET.ParseError as e:
        log(f"❌ Gagal parsing XML: {e}")
        return []

    streams_data = []
    sim_obj_root = root.find("SimulationObjects")
    if sim_obj_root is None:
        log("⚠ Tidak ditemukan tag SimulationObjects di XML.")
        return []

    for stream in sim_obj_root.findall("MaterialStream"):
        name = stream.findtext("Name", "Unknown")
        temp = stream.findtext("Temperature", "0")
        press = stream.findtext("Pressure", "0")
        flow = stream.findtext("MassFlow", "0")

        try:
            temp = float(temp)
            press = float(press)
            flow = float(flow)
        except ValueError:
            temp = press = flow = 0.0

        comps = {}
        comps_root = stream.find("Phases/0/Compounds")
        if comps_root is not None:
            for comp in comps_root.findall("Compound"):
                cname = comp.findtext("Name", "Unknown")
                frac = comp.findtext("MoleFraction", "0")
                try:
                    comps[cname] = float(frac)
                except ValueError:
                    comps[cname] = 0.0

        streams_data.append({
            "name": name,
            "temperature": temp,
            "pressure": press,
            "mass_flow": flow,
            "composition": comps
        })

    return streams_data

# =====================================================
# KIRIM DATA KE INFLUXDB
# =====================================================
def send_to_influxdb(client, data):
    if not data:
        log("⚠ Tidak ada data stream untuk dikirim ke InfluxDB.")
        return

    write_api = client.write_api()
    for s in data:
        point = (
            Point("dwsim_stream")
            .tag("name", s["name"])
            .field("temperature", s["temperature"])
            .field("pressure", s["pressure"])
            .field("mass_flow", s["mass_flow"])
        )

        # Tambahkan komposisi (jika ada)
        for cname, frac in s["composition"].items():
            point.field(f"comp_{cname}", frac)

        write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
        log(f"✅ Data stream '{s['name']}' dikirim ke InfluxDB.")

# =====================================================
# MAIN PROGRAM
# =====================================================
def main():
    log("🚀 Memulai pembacaan DWSIM ke InfluxDB...")
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)

    while True:
        if not os.path.exists(DWSIM_FILE):
            log("⚠ File DWSIM belum ditemukan. Menunggu...")
            time.sleep(POLL_INTERVAL)
            continue

        data = parse_dwsim_xml(DWSIM_FILE)
        send_to_influxdb(client, data)
        time.sleep(POLL_INTERVAL)

if __name__ == "__main__":
    main()



