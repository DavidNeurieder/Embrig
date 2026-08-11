//! Generic deterministic network simulation core.
//!
//! CAN, UDP and TCP share one simulation engine. A [`NetworkSim`] steps
//! [`NetEcu`]s in insertion order every tick, routes outgoing messages through
//! the fault layer and delivers them to subscribed ECUs — the loop each
//! transport used to implement on its own. The transports differ only in the
//! message type and its subscription key: `u32` frame ids for CAN, destination
//! endpoints for UDP/TCP.
//!
//! The same unification applies to firmware: [`NetRegistry`] is the single
//! firmware-factory registry (replacing the CAN `SilRegistry`, the UDP
//! `UdpRegistry` and the TCP `TcpRegistry`), and [`NetEcuError`] is the single
//! ECU error type.

use std::collections::BTreeMap;
use std::time::Instant;

use crate::fault::Fault;
use crate::frame::CanFrame;
use crate::signal::SignalValue;
use crate::time::Timestamp;

/// A message carried on a simulated network: a CAN frame, UDP datagram or TCP
/// segment. Implementations pin the subscription key `K` the message routes
/// by — a frame id for CAN, a destination endpoint for UDP/TCP.
pub trait NetMessage<K>: Clone + Send {
    /// The subscription key this message routes by.
    fn key(&self) -> K;
    /// Mutable payload bytes, used by corruption faults.
    fn payload_mut(&mut self) -> &mut [u8];
    /// Timestamp, stamped by the simulation clock.
    fn ts(&self) -> Timestamp;
    fn set_ts(&mut self, ts: Timestamp);
    /// Short noun for the message kind, used in fault diagnostics.
    fn noun(&self) -> &'static str;
    /// Stable label for the key, used in fault diagnostics ("0x100",
    /// "10.0.0.2:5000").
    fn label(&self) -> String;
}

/// What to do with a message after fault injection has run.
#[derive(Debug, Clone, PartialEq)]
pub enum NetAction {
    Deliver,
    Drop,
    Delay(Timestamp),
    Corrupt { byte: usize, mask: u8 },
}

/// A fault injected into a simulated network, keyed by message.
///
/// Faults are time-windowed: they apply while `start <= now < start + duration`
/// (or forever when `duration` is `None`). The CAN [`Fault`], UDP `UdpFault`
/// and TCP `TcpFault` types all implement this against their own key type.
pub trait NetFault<K> {
    /// Whether this fault applies to a message with `key`.
    fn matches(&self, key: &K) -> bool;
    /// The action to apply when it matches.
    fn action(&self) -> NetAction;
}

/// A fault rule bound to a time window.
#[derive(Debug, Clone, PartialEq)]
pub struct NetFaultRule<F> {
    pub fault: F,
    pub start: Timestamp,
    pub duration: Option<Timestamp>,
}

impl<F> NetFaultRule<F> {
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

/// A recorded item during a network simulation run.
#[derive(Debug, Clone, PartialEq)]
pub enum NetRecord<M> {
    /// A message that was actually delivered on the network.
    Message(M),
    /// A marker event, e.g. a fault being triggered.
    Event {
        ts: Timestamp,
        source: String,
        message: String,
    },
}

/// Ordered event log for a network simulation run.
#[derive(Debug, Clone, PartialEq)]
pub struct NetRecorder<M> {
    pub records: Vec<NetRecord<M>>,
}

impl<M> Default for NetRecorder<M> {
    fn default() -> Self {
        Self {
            records: Vec::new(),
        }
    }
}

impl<M> NetRecorder<M> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, record: NetRecord<M>) {
        self.records.push(record);
    }

    pub fn message(&mut self, msg: M) {
        self.push(NetRecord::Message(msg));
    }

    pub fn event(&mut self, ts: Timestamp, source: impl Into<String>, message: impl Into<String>) {
        self.push(NetRecord::Event {
            ts,
            source: source.into(),
            message: message.into(),
        });
    }

    /// All delivered messages, most recent first.
    pub fn messages(&self) -> Vec<&M> {
        self.records
            .iter()
            .rev()
            .filter_map(|r| match r {
                NetRecord::Message(m) => Some(m),
                _ => None,
            })
            .collect()
    }

    /// The most recent delivered message with the given key, if any.
    pub fn last_message<K: Ord>(&self, key: &K) -> Option<&M>
    where
        M: NetMessage<K>,
    {
        self.records.iter().rev().find_map(|r| match r {
            NetRecord::Message(m) if &m.key() == key => Some(m),
            _ => None,
        })
    }

    /// Whether any delivered message has the given key.
    pub fn has_message<K: Ord>(&self, key: &K) -> bool
    where
        M: NetMessage<K>,
    {
        self.records
            .iter()
            .any(|r| matches!(r, NetRecord::Message(m) if &m.key() == key))
    }

    pub fn clear(&mut self) {
        self.records.clear();
    }
}

/// Errors produced by virtual ECUs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetEcuError {
    /// This ECU does not support runtime signal/field overrides.
    SignalNotSupported,
    /// No such message on this ECU.
    UnknownMessage(String),
    /// No such field (signal) on this message.
    UnknownField(String),
    /// The value (or symbol) cannot be encoded for this field.
    InvalidValue(String),
    /// No firmware implementation is registered for this SIL ECU.
    NotRegistered(String),
}

impl std::fmt::Display for NetEcuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NetEcuError::SignalNotSupported => {
                write!(f, "signal override not supported by this ECU")
            }
            NetEcuError::UnknownMessage(name) => write!(f, "no message `{name}` on this ECU"),
            NetEcuError::UnknownField(name) => write!(f, "no field `{name}` on this message"),
            NetEcuError::InvalidValue(v) => write!(f, "cannot encode value: {v}"),
            NetEcuError::NotRegistered(name) => {
                write!(f, "no firmware registered for SIL ECU {name}")
            }
        }
    }
}

impl std::error::Error for NetEcuError {}

/// A virtual ECU on any simulated network.
///
/// Implementations are stepped in insertion order every simulation tick.
/// [`NetEcu::update`] is called each tick to advance behaviour and produce
/// outgoing messages; [`NetEcu::on_message`] is called when a subscribed
/// message is delivered. [`NetEcu::set_signal`] and [`NetEcu::set_field`] are
/// the test-stimulus hooks: CAN ECUs override signals (keyed by frame id),
/// message-map ECUs override fields (keyed by message name).
pub trait NetEcu<M>: Send {
    /// Stable name used in reports and error messages.
    fn name(&self) -> &str;

    /// Advance the ECU's internal state to `time`.
    fn update(&mut self, _time: Timestamp, _out: &mut Vec<M>) {}

    /// Handle a received message.
    fn on_message(&mut self, _msg: &M, _time: Timestamp) {}

    /// Override a CAN signal value (used by tests to inject stimulus).
    fn set_signal(
        &mut self,
        _id: u32,
        _signal: &str,
        _value: SignalValue,
    ) -> Result<(), NetEcuError> {
        Err(NetEcuError::SignalNotSupported)
    }

    /// Override a message-map field value (used by tests to inject stimulus).
    fn set_field(
        &mut self,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), NetEcuError> {
        Err(NetEcuError::SignalNotSupported)
    }
}

/// A firmware factory for one ECU node.
///
/// `create` is invoked every time the simulation is built (once at startup and
/// again on each test reset), so firmware state never leaks between tests.
/// `step_budget_us` is the wall-clock budget for one simulated step.
pub trait NetEcuFactory<M>: Send + Sync {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn NetEcu<M>>, NetEcuError>;
}

/// Lets callers register a firmware factory as a plain closure, e.g.
/// `registry.register("controller", |_name, _budget| Ok(Box::new(Firmware::new())))`.
impl<F, M> NetEcuFactory<M> for F
where
    F: Fn(&str, u64) -> Result<Box<dyn NetEcu<M>>, NetEcuError> + Send + Sync + 'static,
    M: 'static,
{
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn NetEcu<M>>, NetEcuError> {
        self(name, step_budget_us)
    }
}

/// A firmware factory registry keyed by the `type: sil` ECU name.
///
/// This is the single registry for CAN, UDP and TCP firmware alike: the CAN
/// stack passes `NetRegistry<CanFrame>`, the UDP stack `NetRegistry<UdpDatagram>`
/// and the TCP stack `NetRegistry<TcpSegment>`.
pub struct NetRegistry<M> {
    factories: std::collections::HashMap<String, Box<dyn NetEcuFactory<M>>>,
}

impl<M> Default for NetRegistry<M> {
    fn default() -> Self {
        Self {
            factories: std::collections::HashMap::new(),
        }
    }
}

impl<M> NetRegistry<M> {
    /// An empty registry: instantiating any SIL ECU fails with
    /// [`NetEcuError::NotRegistered`].
    pub fn new() -> Self {
        Self::default()
    }

    /// Register the firmware for an ECU. `factory` may be any [`NetEcuFactory`],
    /// including a plain closure.
    pub fn register(
        &mut self,
        name: impl Into<String>,
        factory: impl NetEcuFactory<M> + 'static,
    ) -> &mut Self {
        self.factories.insert(name.into(), Box::new(factory));
        self
    }

    /// The registered ECU names, sorted (for diagnostics).
    pub fn names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.factories.keys().cloned().collect();
        names.sort();
        names
    }
}

impl<M> NetEcuFactory<M> for NetRegistry<M>
where
    M: 'static,
{
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn NetEcu<M>>, NetEcuError> {
        let factory = self.factories.get(name).ok_or_else(|| {
            NetEcuError::NotRegistered(format!(
                "`{name}` (registered: {})",
                self.names().join(", ")
            ))
        })?;
        let inner = factory.create(name, step_budget_us)?;
        Ok(Box::new(BudgetedNetEcu::new(inner, step_budget_us)))
    }
}

impl<M> NetEcuFactory<M> for &NetRegistry<M>
where
    M: 'static,
{
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn NetEcu<M>>, NetEcuError> {
        NetRegistry::create(self, name, step_budget_us)
    }
}

/// Wall-clock budget enforcement around one firmware ECU.
///
/// Panics if a single `update`/`on_message` call takes longer than
/// `budget_us`; the SIL targets convert that panic into a test failure. The
/// simulation is rebuilt per test, so a panicked step discards the firmware
/// state and the next test starts clean.
struct BudgetedNetEcu<M> {
    inner: Box<dyn NetEcu<M>>,
    budget_us: u64,
    step_start: Instant,
}

impl<M> BudgetedNetEcu<M> {
    fn new(inner: Box<dyn NetEcu<M>>, budget_us: u64) -> Self {
        Self {
            inner,
            budget_us,
            step_start: Instant::now(),
        }
    }
}

impl<M> NetEcu<M> for BudgetedNetEcu<M> {
    fn name(&self) -> &str {
        self.inner.name()
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<M>) {
        self.step_start = Instant::now();
        self.inner.update(time, out);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn on_message(&mut self, msg: &M, time: Timestamp) {
        self.step_start = Instant::now();
        self.inner.on_message(msg, time);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn set_signal(&mut self, id: u32, signal: &str, value: SignalValue) -> Result<(), NetEcuError> {
        self.inner.set_signal(id, signal, value)
    }

    fn set_field(
        &mut self,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        self.inner.set_field(message, field, value)
    }
}

fn check_budget(name: &str, budget_us: u64, start: Instant) {
    let took_us = start.elapsed().as_micros() as u64;
    if took_us > budget_us {
        panic!("firmware `{name}` exceeded its {budget_us}µs step budget (took {took_us}µs)");
    }
}

/// A deterministic virtual network simulation, generic over the message type.
///
/// All ECUs are stepped in insertion order every tick. Time advances in fixed
/// integer-microsecond steps. There is no randomness and no wall clock. CAN
/// frames, UDP datagrams and TCP segments each instantiate this type with
/// their own message and subscription key; the CAN [`Simulation`] is the
/// `u32`-keyed case.
pub struct NetworkSim<K, M, F> {
    time: Timestamp,
    step_us: Timestamp,
    ecus: Vec<Box<dyn NetEcu<M>>>,
    /// key -> indices of subscribed ECUs.
    subscriptions: BTreeMap<K, Vec<usize>>,
    outbound: Vec<M>,
    delayed: Vec<(Timestamp, M)>,
    recorder: NetRecorder<M>,
    faults: Vec<NetFaultRule<F>>,
}

impl<K, M, F> NetworkSim<K, M, F>
where
    K: Ord + Clone + Send + 'static,
    M: NetMessage<K> + 'static,
    F: NetFault<K>,
{
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            time: 0,
            step_us,
            ecus: Vec::new(),
            subscriptions: BTreeMap::new(),
            outbound: Vec::new(),
            delayed: Vec::new(),
            recorder: NetRecorder::new(),
            faults: Vec::new(),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.time
    }

    pub fn recorder(&self) -> &NetRecorder<M> {
        &self.recorder
    }

    pub fn recorder_mut(&mut self) -> &mut NetRecorder<M> {
        &mut self.recorder
    }

    /// Add an ECU. `keys` lists the message keys it wants to receive.
    pub fn attach(&mut self, ecu: Box<dyn NetEcu<M>>, keys: &[K]) -> usize {
        let index = self.ecus.len();
        for key in keys {
            self.subscriptions
                .entry(key.clone())
                .or_default()
                .push(index);
        }
        self.ecus.push(ecu);
        index
    }

    /// Add an ECU subscribed to a single key.
    pub fn attach1(&mut self, ecu: Box<dyn NetEcu<M>>, key: K) -> usize {
        let index = self.ecus.len();
        self.subscriptions.entry(key).or_default().push(index);
        self.ecus.push(ecu);
        index
    }

    pub fn subscribe(&mut self, key: K, ecu_index: usize) {
        self.subscriptions.entry(key).or_default().push(ecu_index);
    }

    pub fn add_fault(&mut self, rule: NetFaultRule<F>) {
        self.faults.push(rule);
    }

    pub fn set_signal(
        &mut self,
        ecu_index: usize,
        id: u32,
        signal: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        self.ecus[ecu_index].set_signal(id, signal, value)
    }

    pub fn set_field(
        &mut self,
        ecu_index: usize,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        self.ecus[ecu_index].set_field(message, field, value)
    }

    /// Inject a message into the network as if an external node sent it.
    pub fn inject(&mut self, mut msg: M) {
        msg.set_ts(self.time);
        self.recorder.message(msg.clone());
        self.deliver(&msg);
    }

    /// Advance the simulation by one tick.
    ///
    /// Each tick runs [`NetEcu::update`] at the current time, routes the
    /// resulting messages through the fault layer, then advances the clock by
    /// `step_us`. A `run_until(t)` therefore processes ticks at times
    /// `0, step, 2·step, …` strictly below `t`, ending with the clock exactly
    /// at `t`.
    pub fn step(&mut self) {
        let mut outbound = std::mem::take(&mut self.outbound);
        for ecu in self.ecus.iter_mut() {
            ecu.update(self.time, &mut outbound);
        }

        for msg in outbound {
            let mut msg = msg;
            msg.set_ts(self.time);
            self.route(msg);
        }

        // Deliver messages whose delay window has elapsed.
        let mut due: Vec<M> = Vec::new();
        self.delayed.retain(|(at, m)| {
            if *at <= self.time {
                due.push(m.clone());
                false
            } else {
                true
            }
        });
        for msg in due {
            self.recorder.message(msg.clone());
            self.deliver(&msg);
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
        self.run_for(ms * crate::time::US_PER_MS);
    }

    fn route(&mut self, mut msg: M) {
        match self.apply_faults(&msg.key()) {
            NetAction::Deliver => {
                self.recorder.message(msg.clone());
                self.deliver(&msg);
            }
            NetAction::Drop => {
                self.recorder.event(
                    self.time,
                    "net",
                    format!("dropped {} {}", msg.noun(), msg.label()),
                );
            }
            NetAction::Delay(delay) => {
                self.recorder.event(
                    self.time,
                    "net",
                    format!("delayed {} {} by {delay}us", msg.noun(), msg.label()),
                );
                let ts = msg.ts() + delay;
                msg.set_ts(ts);
                self.delayed.push((self.time + delay, msg));
            }
            NetAction::Corrupt { byte, mask } => {
                if let Some(b) = msg.payload_mut().get_mut(byte) {
                    *b ^= mask;
                }
                self.recorder.event(
                    self.time,
                    "net",
                    format!("corrupted byte {byte} of {} {}", msg.noun(), msg.label()),
                );
                self.recorder.message(msg.clone());
                self.deliver(&msg);
            }
        }
    }

    fn apply_faults(&self, key: &K) -> NetAction {
        for rule in &self.faults {
            if !rule.active_at(self.time) {
                continue;
            }
            if rule.fault.matches(key) {
                return rule.fault.action();
            }
        }
        NetAction::Deliver
    }

    fn deliver(&mut self, msg: &M) {
        if let Some(indices) = self.subscriptions.get(&msg.key()) {
            for &i in indices {
                self.ecus[i].on_message(msg, self.time);
            }
        }
    }
}

/// CAN-specific conveniences on top of the generic engine.
pub trait CanSimExt {
    /// Frame count per id, ordered by id (deterministic).
    fn frame_counts(&self) -> Vec<(u32, usize)>;
}

impl CanSimExt for NetworkSim<u32, CanFrame, Fault> {
    fn frame_counts(&self) -> Vec<(u32, usize)> {
        let mut counts: BTreeMap<u32, usize> = BTreeMap::new();
        for msg in self.recorder().messages() {
            *counts.entry(msg.key()).or_default() += 1;
        }
        counts.into_iter().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::{ms, US_PER_MS};

    /// Transmits one message every period.
    struct PeriodicSender {
        name: &'static str,
        key: u32,
        period: Timestamp,
        next: Timestamp,
    }

    impl NetEcu<CanFrame> for PeriodicSender {
        fn name(&self) -> &str {
            self.name
        }
        fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
            if time >= self.next {
                out.push(CanFrame::new(self.key, vec![0; 8]).unwrap());
                self.next = time + self.period;
            }
        }
    }

    /// Emits one `listen + 0x100` message per received message.
    struct EchoOnUpdate {
        listen: u32,
        seen: u32,
        emitted: u32,
    }

    impl NetEcu<CanFrame> for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_message(&mut self, msg: &CanFrame, _time: Timestamp) {
            if msg.key() == self.listen {
                self.seen += 1;
            }
        }
        fn update(&mut self, _time: Timestamp, out: &mut Vec<CanFrame>) {
            while self.emitted < self.seen {
                out.push(CanFrame::new(self.listen + 0x100, vec![0; 8]).unwrap());
                self.emitted += 1;
            }
        }
    }

    type Sim = NetworkSim<u32, CanFrame, Fault>;

    fn echo_sim() -> (Sim, usize, usize) {
        let mut sim = Sim::new(US_PER_MS);
        let tx = sim.attach(
            Box::new(PeriodicSender {
                name: "tx",
                key: 0x100,
                period: ms(10),
                next: 0,
            }),
            &[],
        );
        let rx = sim.attach(
            Box::new(EchoOnUpdate {
                listen: 0x100,
                seen: 0,
                emitted: 0,
            }),
            &[0x100],
        );
        (sim, tx, rx)
    }

    #[test]
    fn routing_delivers_only_subscribed_keys() {
        let (mut sim, _tx, _rx) = echo_sim();
        sim.run_ms(110);
        assert_eq!(
            sim.recorder()
                .messages()
                .iter()
                .filter(|m| m.key() == 0x100)
                .count(),
            11
        );
        assert_eq!(
            sim.recorder()
                .messages()
                .iter()
                .filter(|m| m.key() == 0x200)
                .count(),
            11
        );
    }

    #[test]
    fn unsubscribed_keys_are_not_delivered() {
        let mut sim = Sim::new(US_PER_MS);
        sim.attach(
            Box::new(EchoOnUpdate {
                listen: 0x100,
                seen: 0,
                emitted: 0,
            }),
            &[0x100],
        );
        sim.inject(CanFrame::new(0x500, vec![0; 8]).unwrap());
        sim.step();
        assert!(sim.recorder().has_message(&0x500));
        assert!(!sim.recorder().has_message(&0x600));
    }

    #[test]
    fn clock_advances_deterministically() {
        let (mut sim, _, _) = echo_sim();
        sim.run_ms(250);
        assert_eq!(sim.time(), ms(250));
        sim.run_for(ms(250));
        assert_eq!(sim.time(), ms(500));
    }

    #[test]
    fn drop_fault_suppresses_messages_then_recovers() {
        let (mut sim, _, _) = echo_sim();
        sim.add_fault(NetFaultRule {
            fault: Fault::DropFrame { id: 0x100 },
            start: ms(20),
            duration: Some(ms(20)),
        });
        sim.run_ms(50);
        assert_eq!(
            sim.recorder()
                .messages()
                .iter()
                .filter(|m| m.key() == 0x100)
                .count(),
            3
        );
        let drops = sim
            .recorder()
            .records
            .iter()
            .filter(
                |r| matches!(r, NetRecord::Event { message, .. } if message.contains("dropped")),
            )
            .count();
        assert_eq!(drops, 2);
    }

    #[test]
    fn delay_fault_holds_message_back() {
        let mut sim = Sim::new(US_PER_MS);
        sim.attach(
            Box::new(PeriodicSender {
                name: "tx",
                key: 0x100,
                period: ms(10),
                next: 0,
            }),
            &[],
        );
        sim.add_fault(NetFaultRule {
            fault: Fault::DelayFrame {
                id: 0x100,
                delay_us: ms(5),
            },
            start: 0,
            duration: Some(ms(20)),
        });
        sim.run_ms(30);
        let frames: Vec<u64> = sim
            .recorder()
            .messages()
            .iter()
            .filter(|m| m.key() == 0x100)
            .map(|m| m.ts())
            .collect();
        assert_eq!(*frames.last().unwrap(), ms(5));
        assert_eq!(frames.len(), 3);
    }

    #[test]
    fn injection_is_recorded_and_delivered() {
        let mut sim = Sim::new(US_PER_MS);
        sim.attach(
            Box::new(EchoOnUpdate {
                listen: 0x200,
                seen: 0,
                emitted: 0,
            }),
            &[0x200],
        );
        sim.inject(CanFrame::new(0x200, vec![1, 0, 0, 0, 0, 0, 0, 0]).unwrap());
        assert!(sim.recorder().has_message(&0x200));
    }

    #[test]
    fn frame_counts_are_ordered_by_id() {
        let (mut sim, _, _) = echo_sim();
        sim.run_ms(10);
        assert_eq!(sim.frame_counts(), vec![(0x100, 1), (0x200, 1)]);
    }

    #[test]
    fn registry_is_keyed_by_name_and_rerun_per_create() {
        let mut registry = NetRegistry::<CanFrame>::new();
        struct Dummy;
        impl NetEcu<CanFrame> for Dummy {
            fn name(&self) -> &str {
                "dummy"
            }
        }
        registry.register(
            "controller",
            |_name: &str, _budget: u64| -> Result<Box<dyn NetEcu<CanFrame>>, NetEcuError> {
                Ok(Box::new(Dummy))
            },
        );
        let ecu = registry.create("controller", 100_000).unwrap();
        assert_eq!(ecu.name(), "dummy");
        let err = match registry.create("nope", 100_000) {
            Err(e) => e,
            Ok(_) => panic!("expected an unknown-name error"),
        };
        assert!(
            err.to_string()
                .contains("no firmware registered for SIL ECU `nope`"),
            "got: {err}"
        );
    }

    #[test]
    fn budgeted_ecu_rejects_default_overrides() {
        struct Noop;
        impl NetEcu<CanFrame> for Noop {
            fn name(&self) -> &str {
                "noop"
            }
        }
        let mut ecu = Noop;
        assert_eq!(
            ecu.set_signal(0x100, "x", SignalValue::Num(1.0)),
            Err(NetEcuError::SignalNotSupported)
        );
        assert_eq!(
            ecu.set_field("M", "x", SignalValue::Num(1.0)),
            Err(NetEcuError::SignalNotSupported)
        );
    }
}
