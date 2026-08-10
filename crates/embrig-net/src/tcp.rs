//! A deterministic virtual TCP network: the third-transport proof.
//!
//! Everything above the transport is reused unchanged from the UDP stack —
//! the netmap [`MessageDef`] field codec, the config-driven stimulus node and
//! the firmware factory registry (the `SilRegistry`/`UdpRegistry` pattern).
//! What differs is the transport itself: messages travel on *connections*
//! between a source and a destination endpoint, delivered reliably and in
//! order. The sim still steps ECUs deterministically in insertion order with
//! integer-microsecond time, so a TCP netmap suite behaves exactly like its
//! UDP counterpart.
//!
//! ```rust,ignore
//! let netmap: Netmap = /* message name -> MessageDef keyed by dst */;
//! let mut sim = TcpSim::new(1000);
//! sim.attach(Box::new(TcpConfigEcu::new("motion", host, "MotionState",
//!     netmap.message("MotionState").unwrap().clone(), 50_000, base)), host);
//! sim.run_ms(150);
//! let seg = sim.recorder().last_segment(dst).unwrap();
//! let speed = netmap.message("MotionState").unwrap()
//!     .decode_field(&seg.payload, "speed").unwrap();
//! ```

use std::collections::BTreeMap;
use std::net::SocketAddr;

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

/// What to do with a segment after fault injection has run.
#[derive(Debug, Clone, PartialEq)]
enum Action {
    Deliver,
    Drop,
    Delay(Timestamp),
    Corrupt { byte: usize, mask: u8 },
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

/// A fault rule bound to a time window.
#[derive(Debug, Clone, PartialEq)]
pub struct TcpFaultRule {
    pub fault: TcpFault,
    pub start: Timestamp,
    pub duration: Option<Timestamp>,
}

impl TcpFaultRule {
    /// Whether the rule is active at `now`.
    pub fn active_at(&self, now: Timestamp) -> bool {
        if now < self.start {
            return false;
        }
        match self.duration {
            None => true,
            Some(d) => now < self.start + d,
        }
    }
}

/// A recorded item during a TCP network simulation run.
#[derive(Debug, Clone, PartialEq)]
pub enum TcpRecord {
    /// A segment that was actually delivered on the connection.
    Segment(TcpSegment),
    /// A marker event, e.g. a fault being triggered.
    Event {
        ts: Timestamp,
        source: String,
        message: String,
    },
}

/// Ordered event log for a TCP network simulation run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TcpRecorder {
    pub records: Vec<TcpRecord>,
}

impl TcpRecorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, record: TcpRecord) {
        self.records.push(record);
    }

    pub fn segment(&mut self, seg: TcpSegment) {
        self.push(TcpRecord::Segment(seg));
    }

    pub fn event(&mut self, ts: Timestamp, source: impl Into<String>, message: impl Into<String>) {
        self.push(TcpRecord::Event {
            ts,
            source: source.into(),
            message: message.into(),
        });
    }

    /// All delivered segments, most recent first.
    pub fn segments(&self) -> Vec<&TcpSegment> {
        self.records
            .iter()
            .rev()
            .filter_map(|r| match r {
                TcpRecord::Segment(s) => Some(s),
                _ => None,
            })
            .collect()
    }

    /// The most recent segment delivered to `dst`, if any.
    pub fn last_segment(&self, dst: SocketAddr) -> Option<&TcpSegment> {
        self.records.iter().rev().find_map(|r| match r {
            TcpRecord::Segment(s) if s.dst == dst => Some(s),
            _ => None,
        })
    }
}

/// A virtual TCP ECU.
///
/// Implementations are stepped in insertion order every simulation tick.
/// [`TcpEcu::update`] is called each tick to advance behaviour and produce
/// outbound segments; [`TcpEcu::on_segment`] is called when a segment
/// addressed to this ECU's endpoint is delivered on its connection.
pub trait TcpEcu: Send {
    /// Stable name used in reports and error messages.
    fn name(&self) -> &str;

    /// Advance the ECU's internal state to `time`.
    fn update(&mut self, _time: Timestamp, _out: &mut Vec<TcpSegment>) {}

    /// Handle a segment received on a connection.
    fn on_segment(&mut self, _seg: &TcpSegment, _time: Timestamp) {}

    /// Override a field value (used by tests to inject stimulus).
    fn set_field(
        &mut self,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), TcpEcuError> {
        Err(TcpEcuError::SignalNotSupported)
    }
}

/// Errors produced by TCP ECUs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TcpEcuError {
    /// This ECU does not support runtime field overrides.
    SignalNotSupported,
    /// No such message on this ECU.
    UnknownMessage(String),
    /// No such field on this message.
    UnknownField(String),
    /// The value (or symbol) cannot be encoded for this field.
    InvalidValue(String),
    /// No firmware implementation is registered for this SIL ECU.
    NotRegistered(String),
}

impl std::fmt::Display for TcpEcuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TcpEcuError::SignalNotSupported => {
                write!(f, "field override not supported by this ECU")
            }
            TcpEcuError::UnknownMessage(name) => write!(f, "no message `{name}` on this ECU"),
            TcpEcuError::UnknownField(name) => write!(f, "no field `{name}` on this message"),
            TcpEcuError::InvalidValue(v) => write!(f, "cannot encode value for field: {v}"),
            TcpEcuError::NotRegistered(name) => {
                write!(f, "no firmware registered for SIL TCP ECU {name}")
            }
        }
    }
}

impl std::error::Error for TcpEcuError {}

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

    fn encode(&self) -> Result<Vec<u8>, TcpEcuError> {
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
            .map_err(|e| TcpEcuError::InvalidValue(e.to_string()))
    }
}

impl TcpEcu for TcpConfigEcu {
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
    ) -> Result<(), TcpEcuError> {
        if message != self.message_name {
            return Err(TcpEcuError::UnknownMessage(message.to_string()));
        }
        if !self.message.fields.contains_key(field) {
            return Err(TcpEcuError::UnknownField(field.to_string()));
        }
        self.overrides.insert(field.to_string(), value);
        Ok(())
    }
}

/// A firmware factory keyed by the `tcp-sil` ECU name.
///
/// Factories are looked up (and re-invoked) every time the simulation is
/// built, so firmware state never leaks between tests.
pub trait TcpEcuFactory: Send + Sync {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn TcpEcu>, TcpEcuError>;
}

/// A factory registry with no firmware: instantiating any `tcp-sil` ECU fails.
#[derive(Default)]
pub struct NoTcpFirmware;

impl TcpEcuFactory for NoTcpFirmware {
    fn create(&self, name: &str, _step_budget_us: u64) -> Result<Box<dyn TcpEcu>, TcpEcuError> {
        Err(TcpEcuError::NotRegistered(name.to_string()))
    }
}

/// Lets callers register a firmware factory as a plain closure.
impl<F> TcpEcuFactory for F
where
    F: Fn(&str, u64) -> Result<Box<dyn TcpEcu>, TcpEcuError> + Send + Sync + 'static,
{
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn TcpEcu>, TcpEcuError> {
        self(name, step_budget_us)
    }
}

/// A firmware factory registry keyed by the `tcp-sil` ECU name.
#[derive(Default)]
pub struct TcpRegistry {
    factories: BTreeMap<String, Box<dyn TcpEcuFactory>>,
}

impl TcpRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register the firmware for an ECU.
    pub fn register(
        &mut self,
        name: impl Into<String>,
        factory: impl TcpEcuFactory + 'static,
    ) -> &mut Self {
        self.factories.insert(name.into(), Box::new(factory));
        self
    }

    /// The registered ECU names, sorted (for diagnostics).
    pub fn names(&self) -> Vec<String> {
        self.factories.keys().cloned().collect()
    }
}

impl TcpEcuFactory for TcpRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn TcpEcu>, TcpEcuError> {
        let factory = self.factories.get(name).ok_or_else(|| {
            TcpEcuError::NotRegistered(format!(
                "`{name}` (registered: {})",
                self.names().join(", ")
            ))
        })?;
        factory.create(name, step_budget_us)
    }
}

impl TcpEcuFactory for &TcpRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn TcpEcu>, TcpEcuError> {
        TcpRegistry::create(self, name, step_budget_us)
    }
}

/// A deterministic virtual TCP network simulation.
///
/// Mirrors [`crate::sim::UdpSim`] for CAN-style UDP: all ECUs are stepped in
/// insertion order every tick, time advances in fixed integer-microsecond
/// steps, and there is no randomness and no wall clock. Connections are keyed
/// by the destination endpoint (the netmap's `dst`), so the same netmap and
/// the same fault concepts apply unchanged.
pub struct TcpSim {
    time: Timestamp,
    step_us: Timestamp,
    ecus: Vec<Box<dyn TcpEcu>>,
    /// dst endpoint -> indices of connected ECUs.
    connections: BTreeMap<SocketAddr, Vec<usize>>,
    outbound: Vec<TcpSegment>,
    delayed: Vec<(Timestamp, TcpSegment)>,
    recorder: TcpRecorder,
    faults: Vec<TcpFaultRule>,
}

impl TcpSim {
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            time: 0,
            step_us,
            ecus: Vec::new(),
            connections: BTreeMap::new(),
            outbound: Vec::new(),
            delayed: Vec::new(),
            recorder: TcpRecorder::new(),
            faults: Vec::new(),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.time
    }

    pub fn recorder(&self) -> &TcpRecorder {
        &self.recorder
    }

    pub fn recorder_mut(&mut self) -> &mut TcpRecorder {
        &mut self.recorder
    }

    /// Open a connection: segments addressed to `address` are delivered to
    /// the ECU.
    pub fn connect(&mut self, ecu: Box<dyn TcpEcu>, address: SocketAddr) -> usize {
        let index = self.ecus.len();
        self.connections.entry(address).or_default().push(index);
        self.ecus.push(ecu);
        index
    }

    pub fn add_fault(&mut self, rule: TcpFaultRule) {
        self.faults.push(rule);
    }

    pub fn set_field(
        &mut self,
        ecu_index: usize,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), TcpEcuError> {
        self.ecus[ecu_index].set_field(message, field, value)
    }

    /// Inject a segment into the network as if an external node sent it.
    pub fn inject(&mut self, mut seg: TcpSegment) {
        seg.ts = self.time;
        self.recorder.segment(seg.clone());
        self.deliver(&seg);
    }

    /// Advance the simulation by one tick.
    pub fn step(&mut self) {
        let mut outbound = std::mem::take(&mut self.outbound);
        for ecu in self.ecus.iter_mut() {
            ecu.update(self.time, &mut outbound);
        }

        for seg in outbound {
            let mut seg = seg;
            seg.ts = self.time;
            self.route(seg);
        }

        // Deliver segments whose delay window has elapsed.
        let mut due: Vec<TcpSegment> = Vec::new();
        self.delayed.retain(|(at, s)| {
            if *at <= self.time {
                due.push(s.clone());
                false
            } else {
                true
            }
        });
        for seg in due {
            self.recorder.segment(seg.clone());
            self.deliver(&seg);
        }

        self.time += self.step_us;
    }

    /// Run until `until` (µs) has elapsed.
    pub fn run_until(&mut self, until: Timestamp) {
        while self.time < until {
            self.step();
        }
    }

    /// Run for `duration` (µs) from the current time.
    pub fn run_for(&mut self, duration: Timestamp) {
        self.run_until(self.time + duration);
    }

    /// Run for a wall-clock-equivalent `duration` given in milliseconds.
    pub fn run_ms(&mut self, ms: u64) {
        self.run_for(ms * embrig_core::time::US_PER_MS);
    }

    fn route(&mut self, mut seg: TcpSegment) {
        match self.apply_faults(&seg) {
            Action::Deliver => {
                self.recorder.segment(seg.clone());
                self.deliver(&seg);
            }
            Action::Drop => {
                self.recorder
                    .event(self.time, "tcp", format!("dropped segment for {}", seg.dst));
            }
            Action::Delay(delay) => {
                self.recorder.event(
                    self.time,
                    "tcp",
                    format!("delayed segment for {} by {delay}us", seg.dst),
                );
                seg.ts += delay;
                self.delayed.push((self.time + delay, seg));
            }
            Action::Corrupt { byte, mask } => {
                if let Some(b) = seg.payload.get_mut(byte) {
                    *b ^= mask;
                }
                self.recorder.event(
                    self.time,
                    "tcp",
                    format!("corrupted byte {byte} for {}", seg.dst),
                );
                self.recorder.segment(seg.clone());
                self.deliver(&seg);
            }
        }
    }

    fn apply_faults(&self, seg: &TcpSegment) -> Action {
        for rule in &self.faults {
            if !rule.active_at(self.time) {
                continue;
            }
            match rule.fault {
                TcpFault::Drop { dst } if dst == seg.dst => return Action::Drop,
                TcpFault::Delay { dst, delay_us } if dst == seg.dst => {
                    return Action::Delay(delay_us)
                }
                TcpFault::CorruptByte { dst, byte, mask } if dst == seg.dst => {
                    return Action::Corrupt { byte, mask }
                }
                _ => {}
            }
        }
        Action::Deliver
    }

    fn deliver(&mut self, seg: &TcpSegment) {
        if let Some(indices) = self.connections.get(&seg.dst) {
            for &i in indices {
                self.ecus[i].on_segment(seg, self.time);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_core::time::{ms, US_PER_MS};

    /// Transmits one segment to `dst` every period.
    struct PeriodicSender {
        name: &'static str,
        src: SocketAddr,
        dst: SocketAddr,
        period: Timestamp,
        next: Timestamp,
    }

    impl TcpEcu for PeriodicSender {
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

    impl TcpEcu for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_segment(&mut self, seg: &TcpSegment, _time: Timestamp) {
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
            .segments()
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
            .segments()
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
            .segments()
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
        assert!(sim.recorder().last_segment(b).is_some());
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
        let err = match TcpRegistry::new().create("motion", 100_000) {
            Ok(_) => panic!("expected an error"),
            Err(e) => e,
        };
        let message = err.to_string();
        assert!(
            message.contains("no firmware registered for SIL TCP ECU `motion`"),
            "got: {message}"
        );
    }

    #[test]
    fn closure_factory_is_a_factory() {
        struct Dummy;
        impl TcpEcu for Dummy {
            fn name(&self) -> &str {
                "dummy"
            }
        }
        let mut registry = TcpRegistry::new();
        registry.register(
            "motion",
            |_name: &str, _budget: u64| -> Result<Box<dyn TcpEcu>, TcpEcuError> {
                Ok(Box::new(Dummy))
            },
        );
        let ecu = registry.create("motion", 100_000).unwrap();
        assert_eq!(ecu.name(), "dummy");
    }
}
