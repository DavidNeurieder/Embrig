//! CANopen controller firmware for the software-in-the-loop example.
//!
//! This is the code under test — a hand-rolled CANopen node (node id 1, no
//! third-party protocol stack) that stands in for firmware you would compile
//! for a target board. It implements the [`NetEcu`] trait against the virtual
//! bus and speaks just enough CiA 301 for the demo:
//!
//! * **NMT** — starts in PRE_OPERATIONAL; an NMT START (`0x000`, node 1)
//!   moves it to OPERATIONAL; NMT STOP back to STOPPED.
//! * **RPDO1** (`0x201`) — receives the process temperature.
//! * **TPDO1** (`0x181`) — transmits the fail-safe `valve_open` command.
//! * **Heartbeat** (`0x701`) — producer frame carrying the NMT state.
//!
//! It lives in its own crate so it compiles separately from the host harness
//! (`cargo build -p fw-canopen-controller`); the harness in
//! `crates/embrig-sil/examples/sil_canopen.rs` only registers it by name.

use std::sync::OnceLock;

use embrig_canopen::codec::CanOpenCodec;
use embrig_canopen::spec::EcuSpec;
use embrig_canopen::SignalCodec;
use embrig_canopen::{
    heartbeat, nmt, rpdo1, tpdo1, HB_OPERATIONAL, HB_PRE_OPERATIONAL, HB_STOPPED,
    NMT_PRE_OPERATIONAL, NMT_START, NMT_STOP,
};
use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_core::NetEcu;

const EDS: &str = include_str!("../../../crates/embrig-sil/examples/canopen/eds.yaml");

fn codec() -> &'static CanOpenCodec {
    static CODEC: OnceLock<CanOpenCodec> = OnceLock::new();
    CODEC.get_or_init(|| {
        CanOpenCodec::new(&EcuSpec::parse(EDS).expect("valid EDS")).expect("valid node id")
    })
}

/// A minimal CANopen node: opens the valve while the process temperature is
/// within 10..90 °C (fail-safe), but only while it is OPERATIONAL.
pub struct CanOpenControllerFirmware {
    name: String,
    state: u8,
    temperature: f64,
    open: bool,
    next_tpdo: Timestamp,
    next_heartbeat: Timestamp,
    period_us: u64,
    heartbeat_us: u64,
}

impl CanOpenControllerFirmware {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            state: HB_PRE_OPERATIONAL,
            temperature: 0.0,
            open: false,
            next_tpdo: 0,
            next_heartbeat: 0,
            period_us: 50_000,
            heartbeat_us: 100_000,
        }
    }
}

impl NetEcu<CanFrame> for CanOpenControllerFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        let node = codec();
        let nmt_msg = node.message_by_id(nmt()).unwrap();
        if frame.id == nmt() {
            let Ok(node_id) = nmt_msg.decode_signal(&frame.data, "node") else {
                return;
            };
            if node_id.round() as u8 != node.node_id() {
                return;
            }
            let Ok(command) = nmt_msg.decode_signal(&frame.data, "command") else {
                return;
            };
            self.state = match command.round() as u8 {
                NMT_START => {
                    self.open = (10.0..=90.0).contains(&self.temperature);
                    HB_OPERATIONAL
                }
                NMT_STOP => HB_STOPPED,
                NMT_PRE_OPERATIONAL => HB_PRE_OPERATIONAL,
                _ => return,
            };
        } else if frame.id == rpdo1(node.node_id()) {
            let rpdo = node.message_by_id(rpdo1(node.node_id())).unwrap();
            if let Ok(temperature) = rpdo.decode_signal(&frame.data, "temperature") {
                self.temperature = temperature;
                if self.state == HB_OPERATIONAL {
                    self.open = (10.0..=90.0).contains(&self.temperature);
                }
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        let node = codec();
        if time >= self.next_tpdo {
            let data = node
                .message_by_name("TPDO1")
                .unwrap()
                .encode_signals(&[("valve_open", if self.open { 1.0 } else { 0.0 })])
                .expect("valve_open encodes");
            out.push(CanFrame::new(tpdo1(node.node_id()), data).expect("8-byte frame"));
            self.next_tpdo = time + self.period_us;
        }
        if time >= self.next_heartbeat {
            let data = node
                .message_by_name("Heartbeat")
                .unwrap()
                .encode_signals(&[("state", self.state as f64)])
                .expect("state encodes");
            out.push(CanFrame::new(heartbeat(node.node_id()), data).expect("8-byte frame"));
            self.next_heartbeat = time + self.heartbeat_us;
        }
    }
}
