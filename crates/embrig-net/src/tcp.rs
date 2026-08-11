//! A deterministic virtual TCP network: the third-transport proof.
//!
//! Everything above the transport is reused unchanged from the UDP stack —
//! the netmap [`MessageDef`] field codec, the config-driven stimulus node and
//! the firmware factory registry (the [`embrig_core::network::NetRegistry`]
//! pattern). What differs is the transport itself: messages travel on
//! *connections* between a source and a destination endpoint, delivered
//! reliably and in order. The sim still steps ECUs deterministically in
//! insertion order with integer-microsecond time, so a TCP netmap suite
//! behaves exactly like its UDP counterpart. The engine is
//! [`embrig_core::network::NetworkSim`] keyed by the destination endpoint;
//! [`TcpSim`] is a thin TCP-flavoured wrapper.
//!
//! ```rust,ignore
//! let netmap: Netmap = /* message name -> MessageDef keyed by dst */;
//! let mut sim = TcpSim::new(1000);
//! sim.connect(Box::new(TcpConfigEcu::new("motion", host, "MotionState",
//!     netmap.message("MotionState").unwrap().clone(), 50_000, base)), host);
//! sim.run_ms(150);
//! let seg = sim.last_segment(dst).unwrap();
//! let speed = netmap.message("MotionState").unwrap()
//!     .decode_field(&seg.payload, "speed").unwrap();
//! ```

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::network::{
    NetAction, NetEcu, NetEcuError, NetFault, NetFaultRule, NetMessage, NetRecord, NetRecorder,
    NetworkSim,
};
use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;

use crate::netmap::MessageDef;

/// A segment carried on a TCP connection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TcpSegment {
    pub src: SocketAddr,
    pub dst: SocketAddr,
    pub payload: Vec<u8>,
    /// Timestamp when the segment was produced (µs).
    pub ts: Timestamp,
}

impl TcpSegment {
    /// Create a segment with timestamp 0.
    pub fn new(src: SocketAddr, dst: SocketAddr, payload: Vec<u8>) -> Self {
        Self {
            src,
            dst,
            payload,
            ts: 0,
        }
    }
}

impl NetMessage<SocketAddr> for TcpSegment {
    fn key(&self) -> SocketAddr {
        self.dst
    }

    fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.payload
    }

    fn ts(&self) -> Timestamp {
        self.ts
    }

    fn set_ts(&mut self, ts: Timestamp) {
        self.ts = ts;
    }

    fn noun(&self) -> &'static str {
        "segment"
    }

    fn label(&self) -> String {
        format!("{}", self.dst)
    }
}

/// A fault injected into a simulated TCP connection.
///
/// Faults are time-windowed (`start <= now < start + duration`, or forever
/// when `duration` is `None`) and target a connection's destination endpoint,
/// mirroring [`crate::sim::UdpFault`] but at connection granularity.
#[derive(Debug, Clone, PartialEq)]
pub enum TcpFault {
    /// Reset the connection: suppress all segments for the destination.
    Drop { dst: SocketAddr },
    /// Hold back segments for the destination by a fixed amount.
    Delay {
        dst: SocketAddr,
        delay_us: Timestamp,
    },
    /// Flip bits in one byte of segments for the destination.
    CorruptByte {
        dst: SocketAddr,
        byte: usize,
        mask: u8,
    },
}

impl NetFault<SocketAddr> for TcpFault {
    fn matches(&self, key: &SocketAddr) -> bool {
        match self {
            TcpFault::Drop { dst } => dst == key,
            TcpFault::Delay { dst, .. } => dst == key,
            TcpFault::CorruptByte { dst, .. } => dst == key,
        }
    }

    fn action(&self) -> NetAction {
        match self {
            TcpFault::Drop { .. } => NetAction::Drop,
            TcpFault::Delay { delay_us, .. } => NetAction::Delay(*delay_us),
            TcpFault::CorruptByte { byte, mask, .. } => NetAction::Corrupt {
                byte: *byte,
                mask: *mask,
            },
        }
    }
}

/// A fault rule bound to a time window.
pub type TcpFaultRule = NetFaultRule<TcpFault>;

/// A recorded item during a TCP network simulation run.
pub type TcpRecord = NetRecord<TcpSegment>;

/// Ordered event log for a TCP network simulation run.
pub type TcpRecorder = NetRecorder<TcpSegment>;

/// A config-driven stimulus node: transmits one netmap message on a fixed
/// period with field values that can be overridden at runtime. Byte-identical
/// behaviour to [`crate::ecu::UdpConfigEcu`], but framed as TCP segments.
pub struct TcpConfigEcu {
    name: String,
    src: SocketAddr,
    message_name: String,
    message: MessageDef,
    period_us: u64,
    base: BTreeMap<String, SignalValue>,
    overrides: BTreeMap<String, SignalValue>,
    next: Timestamp,
}

impl TcpConfigEcu {
    /// Create a stimulus node. Fields not present in `base` default to zero.
    pub fn new(
        name: String,
        src: SocketAddr,
        message_name: &str,
        message: MessageDef,
        period_us: u64,
        base: BTreeMap<String, SignalValue>,
    ) -> Self {
        let mut base = base;
        for field in message.fields.keys() {
            base.entry(field.clone()).or_insert(SignalValue::Num(0.0));
        }
        Self {
            name,
            src,
            message_name: message_name.to_string(),
            message,
            period_us,
            base,
            overrides: BTreeMap::new(),
            next: 0,
        }
    }

    fn encode(&self) -> Result<Vec<u8>, NetEcuError> {
        let values: Vec<(&str, SignalValue)> = self
            .message
            .fields
            .keys()
            .map(|name| {
                let value = self
                    .overrides
                    .get(name)
                    .or_else(|| self.base.get(name))
                    .cloned()
                    .unwrap_or(SignalValue::Num(0.0));
                (name.as_str(), value)
            })
            .collect();
        self.message
            .encode_fields(&values)
            .map_err(|e| NetEcuError::InvalidValue(e.to_string()))
    }
}

impl NetEcu<TcpSegment> for TcpConfigEcu {
    fn name(&self) -> &str {
        &self.name
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<TcpSegment>) {
        if time < self.next {
            return;
        }
        if let Ok(payload) = self.encode() {
            out.push(TcpSegment::new(self.src, self.message.dst, payload));
        }
        self.next = time + self.period_us;
    }

    fn set_field(
        &mut self,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        if message != self.message_name {
            return Err(NetEcuError::UnknownMessage(message.to_string()));
        }
        if !self.message.fields.contains_key(field) {
            return Err(NetEcuError::UnknownField(field.to_string()));
        }
        self.overrides.insert(field.to_string(), value);
        Ok(())
    }
}

/// A deterministic virtual TCP network simulation.
///
/// Thin wrapper over [`NetworkSim`] keyed by destination endpoint, exposing
/// the TCP-flavoured API (`connect`, `last_segment`).
pub struct TcpSim {
    inner: NetworkSim<SocketAddr, TcpSegment, TcpFault>,
}

impl TcpSim {
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            inner: NetworkSim::new(step_us),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.inner.time()
    }

    pub fn recorder(&self) -> &TcpRecorder {
        self.inner.recorder()
    }

    pub fn recorder_mut(&mut self) -> &mut TcpRecorder {
        self.inner.recorder_mut()
    }

    /// Open a connection: segments addressed to `address` are delivered to
    /// the ECU.
    pub fn connect(&mut self, ecu: Box<dyn NetEcu<TcpSegment>>, address: SocketAddr) -> usize {
        self.inner.attach1(ecu, address)
    }

    pub fn add_fault(&mut self, rule: TcpFaultRule) {
        self.inner.add_fault(rule);
    }

    pub fn set_field(
        &mut self,
        ecu_index: usize,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        self.inner.set_field(ecu_index, message, field, value)
    }

    /// The most recent segment delivered to `dst`, if any.
    pub fn last_segment(&self, dst: SocketAddr) -> Option<&TcpSegment> {
        self.inner.recorder().last_message(&dst)
    }

    /// Inject a segment into the network as if an external node sent it.
    pub fn inject(&mut self, seg: TcpSegment) {
        self.inner.inject(seg);
    }

    /// Advance the simulation by one tick.
    pub fn step(&mut self) {
        self.inner.step();
    }

    /// Run until `until` (µs) has elapsed.
    pub fn run_until(&mut self, until: Timestamp) {
        self.inner.run_until(until);
    }

    /// Run for `duration` (µs) from the current time.
    pub fn run_for(&mut self, duration: Timestamp) {
        self.inner.run_for(duration);
    }

    /// Run for a wall-clock-equivalent `duration` given in milliseconds.
    pub fn run_ms(&mut self, ms: u64) {
        self.inner.run_ms(ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_core::network::NetEcuFactory;
    use embrig_core::time::{ms, US_PER_MS};

    /// Transmits one segment to `dst` every period.
    struct PeriodicSender {
        name: &'static str,
        src: SocketAddr,
        dst: SocketAddr,
        period: Timestamp,
        next: Timestamp,
    }

    impl NetEcu<TcpSegment> for PeriodicSender {
        fn name(&self) -> &str {
            self.name
        }
        fn update(&mut self, time: Timestamp, out: &mut Vec<TcpSegment>) {
            if time >= self.next {
                out.push(TcpSegment::new(self.src, self.dst, vec![0; 8]));
                self.next = time + self.period;
            }
        }
    }

    /// Emits one segment per received segment during the next update.
    struct EchoOnUpdate {
        listen: SocketAddr,
        src: SocketAddr,
        out_dst: SocketAddr,
        seen: u32,
        emitted: u32,
    }

    impl NetEcu<TcpSegment> for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_message(&mut self, seg: &TcpSegment, _time: Timestamp) {
            if seg.dst == self.listen {
                self.seen += 1;
            }
        }
        fn update(&mut self, _time: Timestamp, out: &mut Vec<TcpSegment>) {
            while self.emitted < self.seen {
                out.push(TcpSegment::new(self.src, self.out_dst, vec![0; 8]));
                self.emitted += 1;
            }
        }
    }

    fn net() -> (TcpSim, SocketAddr, SocketAddr) {
        let a: SocketAddr = "10.0.0.1:5000".parse().unwrap();
        let b: SocketAddr = "10.0.0.2:5000".parse().unwrap();
        let mut sim = TcpSim::new(US_PER_MS);
        sim.connect(
            Box::new(PeriodicSender {
                name: "tx",
                src: a,
                dst: b,
                period: ms(10),
                next: 0,
            }),
            a,
        );
        sim.connect(
            Box::new(EchoOnUpdate {
                listen: b,
                src: b,
                out_dst: a,
                seen: 0,
                emitted: 0,
            }),
            b,
        );
        (sim, a, b)
    }

    #[test]
    fn delivery_is_reliable_and_in_order() {
        let (mut sim, _a, b) = net();
        sim.run_ms(105);
        // One segment every 10ms from t=0..100 inclusive -> 11 delivered, in
        // arrival order.
        let delivered: Vec<&TcpSegment> = sim
            .recorder()
            .messages()
            .iter()
            .rev()
            .filter(|s| s.dst == b)
            .copied()
            .collect();
        assert_eq!(delivered.len(), 11);
        for pair in delivered.windows(2) {
            assert!(pair[0].ts <= pair[1].ts, "out of order");
        }
    }

    #[test]
    fn clock_advances_deterministically() {
        let (mut sim, _, _) = net();
        sim.run_ms(250);
        assert_eq!(sim.time(), ms(250));
        sim.run_for(ms(250));
        assert_eq!(sim.time(), ms(500));
    }

    #[test]
    fn drop_fault_resets_then_recovers() {
        let (mut sim, _a, b) = net();
        sim.add_fault(TcpFaultRule {
            fault: TcpFault::Drop { dst: b },
            start: ms(20),
            duration: Some(ms(20)),
        });
        sim.run_ms(50);
        let count = sim
            .recorder()
            .messages()
            .iter()
            .filter(|s| s.dst == b)
            .count();
        // Produced at t=0,10,20,30,40; dropped during [20,40): t=20,30.
        assert_eq!(count, 3);
    }

    #[test]
    fn delay_fault_holds_segment_back() {
        let (mut sim, _a, b) = net();
        sim.add_fault(TcpFaultRule {
            fault: TcpFault::Delay {
                dst: b,
                delay_us: ms(5),
            },
            start: 0,
            duration: Some(ms(20)),
        });
        sim.run_ms(30);
        let dgs: Vec<Timestamp> = sim
            .recorder()
            .messages()
            .iter()
            .filter(|s| s.dst == b)
            .map(|s| s.ts)
            .collect();
        // t=0 and t=10 segments are held back 5ms (delivered at t=5, t=15);
        // the t=20 segment clears the window and is delivered on time.
        assert_eq!(dgs, vec![ms(20), ms(15), ms(5)]);
        assert_eq!(dgs.len(), 3);
    }

    #[test]
    fn injection_is_recorded_and_delivered() {
        let (mut sim, a, b) = net();
        sim.inject(TcpSegment::new(a, b, vec![1, 2, 3, 4, 5, 6, 7, 8]));
        assert!(sim.last_segment(b).is_some());
    }

    fn message() -> MessageDef {
        MessageDef {
            dst: "192.168.1.30:5000".parse().unwrap(),
            length: 4,
            fields: BTreeMap::from([(
                "speed".to_string(),
                crate::netmap::FieldDef {
                    offset: 0,
                    ty: crate::netmap::FieldType::F32le,
                    factor: 1.0,
                    shift: 0.0,
                    values: BTreeMap::new(),
                },
            )]),
        }
    }

    #[test]
    fn config_ecu_emits_and_overrides_using_the_netmap_codec() {
        let src: SocketAddr = "192.168.1.10:5000".parse().unwrap();
        let mut ecu = TcpConfigEcu::new(
            "motion".into(),
            src,
            "MotionState",
            message(),
            100_000,
            BTreeMap::new(),
        );
        let mut out = Vec::new();
        ecu.update(0, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].dst, message().dst);
        assert_eq!(out[0].src, src);

        ecu.set_field("MotionState", "speed", SignalValue::Num(2.5))
            .unwrap();
        let mut out = Vec::new();
        ecu.update(100_000, &mut out);
        let decoded = message().decode_field(&out[0].payload, "speed").unwrap();
        assert!((decoded.value - 2.5).abs() < 1e-6);
    }

    #[test]
    fn registry_unknown_ecu_fails_clearly() {
        let registry = embrig_core::network::NetRegistry::<TcpSegment>::new();
        let err = match registry.create("motion", 100_000) {
            Ok(_) => panic!("expected an error"),
            Err(e) => e,
        };
        let message = err.to_string();
        assert!(
            message.contains("no firmware registered for SIL ECU `motion`"),
            "got: {message}"
        );
    }

    #[test]
    fn closure_factory_is_a_factory() {
        struct Dummy;
        impl NetEcu<TcpSegment> for Dummy {
            fn name(&self) -> &str {
                "dummy"
            }
        }
        let mut registry = embrig_core::network::NetRegistry::<TcpSegment>::new();
        registry.register(
            "motion",
            |_name: &str, _budget: u64| -> Result<Box<dyn NetEcu<TcpSegment>>, NetEcuError> {
                Ok(Box::new(Dummy))
            },
        );
        let ecu = registry.create("motion", 100_000).unwrap();
        assert_eq!(ecu.name(), "dummy");
    }
}
