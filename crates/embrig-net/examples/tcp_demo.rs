//! TCP proof transport demo.
//!
//! Shows that the message-map transport pattern extends beyond UDP: the
//! netmap field codec, the config-driven stimulus node (`TcpConfigEcu`) and
//! the firmware factory registry (the unified `NetRegistry`, also used by the
//! CAN `SilRegistry` and the UDP stack) are all reused unchanged, only the
//!
//! Scenario (deterministic, step 1 ms):
//! * `joystick` (config ECU, host) transmits `DriveCommand` to the `motion`
//!   node every 20 ms.
//! * `motion` (host-compiled firmware via `NetRegistry`) echoes its `speed`
//!   back to the host as `MotionState` every 10 ms.
//! * A windowed `Drop` fault resets the `MotionState` connection during
//!   [40 ms, 80 ms) — the host sees no telemetry, then it recovers.
//!
//! Run with: `cargo run --example tcp_demo --package embrig-net`

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::network::{NetEcu, NetEcuError, NetEcuFactory, NetRegistry};
use embrig_core::signal::SignalValue;
use embrig_core::time::{ms, Timestamp};
use embrig_net::{
    FieldDef, FieldType, MessageDef, Netmap, TcpConfigEcu, TcpFault, TcpFaultRule, TcpSegment,
    TcpSim,
};

const HOST: &str = "192.168.1.10:5000";
const MOTION: &str = "192.168.1.30:5000";
const STEP_US: u64 = 1_000;

fn field(offset: usize, ty: FieldType) -> FieldDef {
    FieldDef {
        offset,
        ty,
        factor: 1.0,
        shift: 0.0,
        values: BTreeMap::new(),
    }
}

/// The netmap is built in code here, but it deserializes from the same YAML
/// shape as the UDP stacks (`netmap.yaml`).
fn netmap() -> Netmap {
    let mut messages = BTreeMap::new();
    messages.insert(
        "DriveCommand".to_string(),
        MessageDef {
            dst: MOTION.parse().unwrap(),
            length: 4,
            fields: BTreeMap::from([("forward".to_string(), field(0, FieldType::F32le))]),
        },
    );
    messages.insert(
        "MotionState".to_string(),
        MessageDef {
            dst: HOST.parse().unwrap(),
            length: 4,
            fields: BTreeMap::from([("speed".to_string(), field(0, FieldType::F32le))]),
        },
    );
    Netmap { messages }
}

/// Firmware under test: echoes the commanded speed as telemetry.
struct MotionFirmware {
    name: String,
    src: SocketAddr,
    out_dst: SocketAddr,
    drive: MessageDef,
    report: MessageDef,
    speed: f64,
    pending: bool,
    next: Timestamp,
}

impl NetEcu<TcpSegment> for MotionFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, seg: &TcpSegment, _time: Timestamp) {
        if let Ok(cmd) = self.drive.decode_field(&seg.payload, "forward") {
            self.speed = cmd.value;
            self.pending = true;
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<TcpSegment>) {
        if self.pending && time >= self.next {
            let payload = self
                .report
                .encode_fields(&[("speed", SignalValue::Num(self.speed))])
                .unwrap();
            out.push(TcpSegment::new(self.src, self.out_dst, payload));
            self.pending = false;
            self.next = time + ms(10);
        }
    }
}

fn main() {
    let host: SocketAddr = HOST.parse().unwrap();
    let motion: SocketAddr = MOTION.parse().unwrap();
    let netmap = netmap();

    let mut sim = TcpSim::new(STEP_US);

    // The `joystick` stimulus node, driven by config like the UDP stack.
    sim.connect(
        Box::new(TcpConfigEcu::new(
            "joystick".into(),
            host,
            "DriveCommand",
            netmap.message("DriveCommand").unwrap().clone(),
            ms(20),
            BTreeMap::from([("forward".to_string(), SignalValue::Num(1.0))]),
        )),
        host,
    );

    // The node under test, bound through the firmware registry.
    let drive = netmap.message("DriveCommand").unwrap().clone();
    let report = netmap.message("MotionState").unwrap().clone();
    let mut firmware = NetRegistry::<TcpSegment>::new();
    firmware.register(
        "motion",
        move |name: &str, _budget: u64| -> Result<Box<dyn NetEcu<TcpSegment>>, NetEcuError> {
            Ok(Box::new(MotionFirmware {
                name: name.to_string(),
                src: motion,
                out_dst: host,
                drive: drive.clone(),
                report: report.clone(),
                speed: 0.0,
                pending: false,
                next: 0,
            }))
        },
    );
    sim.connect(firmware.create("motion", 100_000).unwrap(), motion);

    // Reset the MotionState connection for [40ms, 80ms): two telemetry
    // emissions (t=41ms and t=61ms) are suppressed, then it recovers.
    sim.add_fault(TcpFaultRule {
        fault: TcpFault::Drop { dst: host },
        start: ms(40),
        duration: Some(ms(40)),
    });

    sim.run_ms(150);

    println!("TCP proof transport (deterministic, step {STEP_US}us)");
    let delivered: Vec<&TcpSegment> = sim
        .recorder()
        .messages()
        .into_iter()
        .filter(|s| s.dst == host)
        .collect();
    for seg in delivered.iter().rev() {
        let speed = netmap
            .message("MotionState")
            .unwrap()
            .decode_field(&seg.payload, "speed")
            .unwrap();
        println!(
            "  t={:>4}ms  MotionState speed={:.1}",
            seg.ts / ms(1),
            speed.value
        );
    }
    let drops = sim
        .recorder()
        .records
        .iter()
        .filter(|r| matches!(r, embrig_net::TcpRecord::Event { message, .. } if message.contains("dropped")))
        .count();
    println!("  {drops} segments dropped by the connection fault");
    println!("  {} delivered", delivered.len());

    let recovered_speed = netmap
        .message("MotionState")
        .unwrap()
        .decode_field(&delivered[0].payload, "speed")
        .unwrap()
        .value;
    assert!(
        drops == 2,
        "expected 2 drops during [40ms,80ms), got {drops}"
    );
    assert!(
        (recovered_speed - 1.0).abs() < 1e-6,
        "firmware should keep echoing the commanded speed, got {recovered_speed}"
    );
    println!("PASS: connection drop [40ms,80ms) suppressed 2 segments, then recovered");
}
