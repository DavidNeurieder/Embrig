//! A deterministic virtual UDP network simulation.
//!
//! Mirrors [`embrig_core::simulation::Simulation`] for CAN: all ECUs are
//! stepped in insertion order every tick, time advances in fixed
//! integer-microsecond steps, and there is no randomness and no wall clock.

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;

use crate::datagram::UdpDatagram;
use crate::ecu::{UdpEcu, UdpEcuError};

/// What to do with a datagram after fault injection has run.
#[derive(Debug, Clone, PartialEq)]
enum Action {
    Deliver,
    Drop,
    Delay(Timestamp),
    Corrupt { byte: usize, mask: u8 },
}

/// A fault injected into the simulated network.
///
/// Faults are time-windowed: they apply while
/// `start <= now < start + duration` (or forever when `duration` is `None`).
/// They target a message's destination endpoint.
#[derive(Debug, Clone, PartialEq)]
pub enum UdpFault {
    /// Suppress all datagrams for the destination.
    Drop { dst: SocketAddr },
    /// Delay datagrams for the destination by a fixed amount.
    Delay {
        dst: SocketAddr,
        delay_us: Timestamp,
    },
    /// Flip bits in one byte of datagrams for the destination.
    CorruptByte {
        dst: SocketAddr,
        byte: usize,
        mask: u8,
    },
}

/// A fault rule bound to a time window.
#[derive(Debug, Clone, PartialEq)]
pub struct UdpFaultRule {
    pub fault: UdpFault,
    pub start: Timestamp,
    pub duration: Option<Timestamp>,
}

impl UdpFaultRule {
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
pub enum UdpRecord {
    /// A datagram that was actually delivered on the network.
    Datagram(UdpDatagram),
    /// A marker event, e.g. a fault being triggered.
    Event {
        ts: Timestamp,
        source: String,
        message: String,
    },
}

/// Ordered event log for a network simulation run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UdpRecorder {
    pub records: Vec<UdpRecord>,
}

impl UdpRecorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, record: UdpRecord) {
        self.records.push(record);
    }

    pub fn datagram(&mut self, dg: UdpDatagram) {
        self.push(UdpRecord::Datagram(dg));
    }

    pub fn event(&mut self, ts: Timestamp, source: impl Into<String>, message: impl Into<String>) {
        self.push(UdpRecord::Event {
            ts,
            source: source.into(),
            message: message.into(),
        });
    }

    /// All datagrams, in order, from most recent to least recent.
    pub fn datagrams(&self) -> Vec<&UdpDatagram> {
        self.records
            .iter()
            .rev()
            .filter_map(|r| match r {
                UdpRecord::Datagram(d) => Some(d),
                _ => None,
            })
            .collect()
    }

    /// The most recent datagram delivered to `dst`, if any.
    pub fn last_datagram(&self, dst: SocketAddr) -> Option<&UdpDatagram> {
        self.records.iter().rev().find_map(|r| match r {
            UdpRecord::Datagram(d) if d.dst == dst => Some(d),
            _ => None,
        })
    }
}

/// A deterministic virtual UDP network simulation.
pub struct UdpSim {
    time: Timestamp,
    step_us: Timestamp,
    ecus: Vec<Box<dyn UdpEcu>>,
    /// dst endpoint -> indices of subscribed ECUs.
    subscriptions: BTreeMap<SocketAddr, Vec<usize>>,
    outbound: Vec<UdpDatagram>,
    delayed: Vec<(Timestamp, UdpDatagram)>,
    recorder: UdpRecorder,
    faults: Vec<UdpFaultRule>,
}

impl UdpSim {
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            time: 0,
            step_us,
            ecus: Vec::new(),
            subscriptions: BTreeMap::new(),
            outbound: Vec::new(),
            delayed: Vec::new(),
            recorder: UdpRecorder::new(),
            faults: Vec::new(),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.time
    }

    pub fn recorder(&self) -> &UdpRecorder {
        &self.recorder
    }

    pub fn recorder_mut(&mut self) -> &mut UdpRecorder {
        &mut self.recorder
    }

    /// Add an ECU. Datagrams addressed to `address` are delivered to it.
    pub fn attach(&mut self, ecu: Box<dyn UdpEcu>, address: SocketAddr) -> usize {
        let index = self.ecus.len();
        self.subscriptions.entry(address).or_default().push(index);
        self.ecus.push(ecu);
        index
    }

    pub fn add_fault(&mut self, rule: UdpFaultRule) {
        self.faults.push(rule);
    }

    pub fn set_field(
        &mut self,
        ecu_index: usize,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), UdpEcuError> {
        self.ecus[ecu_index].set_field(message, field, value)
    }

    /// Inject a datagram into the network as if an external node sent it.
    pub fn inject(&mut self, mut dg: UdpDatagram) {
        dg.ts = self.time;
        self.recorder.datagram(dg.clone());
        self.deliver(&dg);
    }

    /// Advance the simulation by one tick.
    pub fn step(&mut self) {
        let mut outbound = std::mem::take(&mut self.outbound);
        for ecu in self.ecus.iter_mut() {
            ecu.update(self.time, &mut outbound);
        }

        for dg in outbound {
            let mut dg = dg;
            dg.ts = self.time;
            self.route(dg);
        }

        // Deliver datagrams whose delay window has elapsed.
        let mut due: Vec<UdpDatagram> = Vec::new();
        self.delayed.retain(|(at, d)| {
            if *at <= self.time {
                due.push(d.clone());
                false
            } else {
                true
            }
        });
        for dg in due {
            self.recorder.datagram(dg.clone());
            self.deliver(&dg);
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

    fn route(&mut self, mut dg: UdpDatagram) {
        match self.apply_faults(&dg) {
            Action::Deliver => {
                self.recorder.datagram(dg.clone());
                self.deliver(&dg);
            }
            Action::Drop => {
                self.recorder
                    .event(self.time, "net", format!("dropped datagram for {}", dg.dst));
            }
            Action::Delay(delay) => {
                self.recorder.event(
                    self.time,
                    "net",
                    format!("delayed datagram for {} by {delay}us", dg.dst),
                );
                dg.ts += delay;
                self.delayed.push((self.time + delay, dg));
            }
            Action::Corrupt { byte, mask } => {
                if let Some(b) = dg.payload.get_mut(byte) {
                    *b ^= mask;
                }
                self.recorder.event(
                    self.time,
                    "net",
                    format!("corrupted byte {byte} for {}", dg.dst),
                );
                self.recorder.datagram(dg.clone());
                self.deliver(&dg);
            }
        }
    }

    fn apply_faults(&self, dg: &UdpDatagram) -> Action {
        for rule in &self.faults {
            if !rule.active_at(self.time) {
                continue;
            }
            match rule.fault {
                UdpFault::Drop { dst } if dst == dg.dst => return Action::Drop,
                UdpFault::Delay { dst, delay_us } if dst == dg.dst => {
                    return Action::Delay(delay_us)
                }
                UdpFault::CorruptByte { dst, byte, mask } if dst == dg.dst => {
                    return Action::Corrupt { byte, mask }
                }
                _ => {}
            }
        }
        Action::Deliver
    }

    fn deliver(&mut self, dg: &UdpDatagram) {
        if let Some(indices) = self.subscriptions.get(&dg.dst) {
            for &i in indices {
                self.ecus[i].on_datagram(dg, self.time);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_core::time::{ms, US_PER_MS};

    /// Transmits one datagram to `dst` every period.
    struct PeriodicSender {
        name: &'static str,
        src: SocketAddr,
        dst: SocketAddr,
        period: Timestamp,
        next: Timestamp,
    }

    impl UdpEcu for PeriodicSender {
        fn name(&self) -> &str {
            self.name
        }
        fn update(&mut self, time: Timestamp, out: &mut Vec<UdpDatagram>) {
            if time >= self.next {
                out.push(UdpDatagram::new(self.src, self.dst, vec![0; 8]));
                self.next = time + self.period;
            }
        }
    }

    /// Emits one datagram per received datagram during the next update.
    struct EchoOnUpdate {
        listen: SocketAddr,
        src: SocketAddr,
        out_dst: SocketAddr,
        seen: u32,
        emitted: u32,
    }

    impl UdpEcu for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_datagram(&mut self, dg: &UdpDatagram, _time: Timestamp) {
            if dg.dst == self.listen {
                self.seen += 1;
            }
        }
        fn update(&mut self, _time: Timestamp, out: &mut Vec<UdpDatagram>) {
            while self.emitted < self.seen {
                out.push(UdpDatagram::new(self.src, self.out_dst, vec![0; 8]));
                self.emitted += 1;
            }
        }
    }

    fn net() -> (UdpSim, SocketAddr, SocketAddr) {
        let a: SocketAddr = "10.0.0.1:5000".parse().unwrap();
        let b: SocketAddr = "10.0.0.2:5000".parse().unwrap();
        let mut sim = UdpSim::new(US_PER_MS);
        sim.attach(
            Box::new(PeriodicSender {
                name: "tx",
                src: a,
                dst: b,
                period: ms(10),
                next: 0,
            }),
            a,
        );
        sim.attach(
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
    fn routing_delivers_only_to_the_destination() {
        let (mut sim, _a, b) = net();
        sim.run_ms(110);
        let count = sim
            .recorder()
            .datagrams()
            .iter()
            .filter(|d| d.dst == b)
            .count();
        // 0x… every 10ms -> 11 transmissions (t=0..100). Each is echoed once,
        // so 11 echoes also land within the run window.
        let echo_count = sim
            .recorder()
            .datagrams()
            .iter()
            .filter(|d| d.dst == _a)
            .count();
        assert_eq!(count, 11);
        assert_eq!(echo_count, 11);
    }

    #[test]
    fn unsubscribed_endpoints_are_recorded_but_not_delivered() {
        let (mut sim, a, _b) = net();
        let c: SocketAddr = "10.0.0.3:5000".parse().unwrap();
        sim.inject(UdpDatagram::new(a, c, vec![1, 2, 3]));
        sim.step();
        assert!(sim.recorder().last_datagram(c).is_some());
        // No ECU subscribes to c, so no echo is produced for it.
        let echoed = sim
            .recorder()
            .datagrams()
            .iter()
            .filter(|d| d.dst == a)
            .count();
        assert_eq!(echoed, 0);
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
    fn drop_fault_suppresses_datagrams_then_recovers() {
        let (mut sim, _a, b) = net();
        sim.add_fault(UdpFaultRule {
            fault: UdpFault::Drop { dst: b },
            start: ms(20),
            duration: Some(ms(20)),
        });
        sim.run_ms(50);
        // Produced at t=0,10,20,30,40. Dropped during [20,40): t=20,30.
        let count = sim
            .recorder()
            .datagrams()
            .iter()
            .filter(|d| d.dst == b)
            .count();
        assert_eq!(count, 3);
        let drops = sim
            .recorder()
            .records
            .iter()
            .filter(
                |r| matches!(r, UdpRecord::Event { message, .. } if message.contains("dropped")),
            )
            .count();
        assert_eq!(drops, 2);
    }

    #[test]
    fn delay_fault_holds_datagram_back() {
        let (mut sim, _a, b) = net();
        sim.add_fault(UdpFaultRule {
            fault: UdpFault::Delay {
                dst: b,
                delay_us: ms(5),
            },
            start: 0,
            duration: Some(ms(20)),
        });
        sim.run_ms(30);
        let dgs: Vec<Timestamp> = sim
            .recorder()
            .datagrams()
            .iter()
            .filter(|d| d.dst == b)
            .map(|d| d.ts)
            .collect();
        // First (t=0) delayed to t=5; frames still produced each 10ms.
        assert_eq!(*dgs.last().unwrap(), ms(5));
        assert_eq!(dgs.len(), 3);
    }

    #[test]
    fn injection_is_recorded_and_delivered() {
        let (mut sim, a, b) = net();
        sim.inject(UdpDatagram::new(a, b, vec![1, 2, 3, 4, 5, 6, 7, 8]));
        assert!(sim.recorder().last_datagram(b).is_some());
    }
}
