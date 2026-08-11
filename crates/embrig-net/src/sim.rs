//! A deterministic virtual UDP network simulation.
//!
//! The engine is [`embrig_core::network::NetworkSim`], shared with CAN and
//! TCP; [`UdpSim`] is a thin UDP-flavoured wrapper: subscriptions are keyed by
//! the destination [`SocketAddr`], ECUs are [`NetEcu<UdpDatagram>`] and faults
//! are [`UdpFault`]. All ECUs step in insertion order every tick, time
//! advances in fixed integer-microsecond steps, and there is no randomness and
//! no wall clock.

use std::net::SocketAddr;

use embrig_core::network::{
    NetAction, NetEcu, NetFault, NetFaultRule, NetRecord, NetRecorder, NetworkSim,
};
use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;

use crate::datagram::UdpDatagram;

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

impl NetFault<SocketAddr> for UdpFault {
    fn matches(&self, key: &SocketAddr) -> bool {
        match self {
            UdpFault::Drop { dst } => dst == key,
            UdpFault::Delay { dst, .. } => dst == key,
            UdpFault::CorruptByte { dst, .. } => dst == key,
        }
    }

    fn action(&self) -> NetAction {
        match self {
            UdpFault::Drop { .. } => NetAction::Drop,
            UdpFault::Delay { delay_us, .. } => NetAction::Delay(*delay_us),
            UdpFault::CorruptByte { byte, mask, .. } => NetAction::Corrupt {
                byte: *byte,
                mask: *mask,
            },
        }
    }
}

/// A UDP fault rule bound to a time window.
pub type UdpFaultRule = NetFaultRule<UdpFault>;

/// A recorded item during a network simulation run.
pub type UdpRecord = NetRecord<UdpDatagram>;

/// Ordered event log for a UDP network simulation run.
pub type UdpRecorder = NetRecorder<UdpDatagram>;

/// A deterministic virtual UDP network simulation.
///
/// Thin wrapper over [`NetworkSim`] keyed by destination endpoint, exposing
/// the UDP-flavoured API (`attach` with a single address, `last_datagram`).
pub struct UdpSim {
    inner: NetworkSim<SocketAddr, UdpDatagram, UdpFault>,
}

impl UdpSim {
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            inner: NetworkSim::new(step_us),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.inner.time()
    }

    pub fn recorder(&self) -> &UdpRecorder {
        self.inner.recorder()
    }

    pub fn recorder_mut(&mut self) -> &mut UdpRecorder {
        self.inner.recorder_mut()
    }

    /// Add an ECU. Datagrams addressed to `address` are delivered to it.
    pub fn attach(&mut self, ecu: Box<dyn NetEcu<UdpDatagram>>, address: SocketAddr) -> usize {
        self.inner.attach1(ecu, address)
    }

    pub fn add_fault(&mut self, rule: UdpFaultRule) {
        self.inner.add_fault(rule);
    }

    pub fn set_field(
        &mut self,
        ecu_index: usize,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), embrig_core::network::NetEcuError> {
        self.inner.set_field(ecu_index, message, field, value)
    }

    /// The most recent datagram delivered to `dst`, if any.
    pub fn last_datagram(&self, dst: SocketAddr) -> Option<&UdpDatagram> {
        self.inner.recorder().last_message(&dst)
    }

    /// Inject a datagram into the network as if an external node sent it.
    pub fn inject(&mut self, dg: UdpDatagram) {
        self.inner.inject(dg);
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
    use embrig_core::time::{ms, US_PER_MS};

    /// Transmits one datagram to `dst` every period.
    struct PeriodicSender {
        name: &'static str,
        src: SocketAddr,
        dst: SocketAddr,
        period: Timestamp,
        next: Timestamp,
    }

    impl NetEcu<UdpDatagram> for PeriodicSender {
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

    impl NetEcu<UdpDatagram> for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_message(&mut self, dg: &UdpDatagram, _time: Timestamp) {
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
            .messages()
            .iter()
            .filter(|d| d.dst == b)
            .count();
        // 0x… every 10ms -> 11 transmissions (t=0..100). Each is echoed once,
        // so 11 echoes also land within the run window.
        let echo_count = sim
            .recorder()
            .messages()
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
        assert!(sim.last_datagram(c).is_some());
        // No ECU subscribes to c, so no echo is produced for it.
        let echoed = sim
            .recorder()
            .messages()
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
            .messages()
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
            .messages()
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
        assert!(sim.last_datagram(b).is_some());
    }
}
