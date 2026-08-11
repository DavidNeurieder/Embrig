//! Pump-controller firmware for the software-in-the-loop example.
//!
//! This is the code under test — a stand-in for firmware you would compile for
//! a target board. It is the **only** embedded code in the SIL example: it
//! lives in its own crate so it compiles separately from the host test
//! harness (`cargo build -p fw-controller`), and the harness in
//! `embrig-sil/examples/sil_firmware.rs` only registers it by name.
//!
//! It implements the [`NetEcu`] trait against the virtual bus:
//!
//! * `on_message` — receives the temperature reading on `0x100`.
//! * `update` — runs the control law (valve open while 10..=90 °C, closed
//!   otherwise, fail-safe) and transmits `valve_open` on `0x200` every 50 ms.

use std::sync::OnceLock;

use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_core::NetEcu;
use embrig_dbc::Network;

const DBC: &str = include_str!("../../../crates/embrig-sil/examples/fixtures/controller.dbc");

fn network() -> &'static Network {
    static NETWORK: OnceLock<Network> = OnceLock::new();
    NETWORK.get_or_init(|| embrig_dbc::parse(DBC).expect("valid controller DBC"))
}

/// The system under test: opens a valve while the temperature is within
/// 10..90 °C, closes it otherwise (fail-safe).
pub struct ControllerFirmware {
    name: String,
    temperature: f64,
    open: bool,
    next_tx: Timestamp,
    period_us: u64,
}

impl ControllerFirmware {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            temperature: 0.0,
            open: false,
            next_tx: 0,
            period_us: 50_000,
        }
    }
}

impl NetEcu<CanFrame> for ControllerFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        if frame.id == 0x100 {
            if let Ok(temperature) = network()
                .message(0x100)
                .unwrap()
                .decode_signal(&frame.data, "temperature")
            {
                self.temperature = temperature;
                self.open = (10.0..=90.0).contains(&self.temperature);
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time >= self.next_tx {
            let data = network()
                .message(0x200)
                .unwrap()
                .encode_signals(&[("valve_open", if self.open { 1.0 } else { 0.0 })])
                .expect("valve_open encodes");
            out.push(CanFrame::new(0x200, data).expect("8-byte frame"));
            self.next_tx = time + self.period_us;
        }
    }
}
