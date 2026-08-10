use std::collections::BTreeMap;

use crate::ecu::{Ecu, EcuError};
use crate::fault::{Fault, FaultRule};
use crate::frame::CanFrame;
use crate::recorder::Recorder;
use crate::signal::SignalValue;
use crate::time::Timestamp;

/// What to do with a frame after fault injection has run.
#[derive(Debug, Clone, PartialEq)]
enum Action {
    Deliver,
    Drop,
    Delay(Timestamp),
    Corrupt { byte: usize, mask: u8 },
}

/// A deterministic virtual CAN network simulation.
///
/// All ECUs are stepped in insertion order every tick. Time advances in fixed
/// integer-microsecond steps. There is no randomness and no wall clock.
pub struct Simulation {
    time: Timestamp,
    step_us: Timestamp,
    ecus: Vec<Box<dyn Ecu>>,
    /// frame id -> indices of subscribed ECUs.
    subscriptions: BTreeMap<u32, Vec<usize>>,
    outbound: Vec<CanFrame>,
    delayed: Vec<(Timestamp, CanFrame)>,
    recorder: Recorder,
    faults: Vec<FaultRule>,
}

impl Simulation {
    pub fn new(step_us: Timestamp) -> Self {
        Self {
            time: 0,
            step_us,
            ecus: Vec::new(),
            subscriptions: BTreeMap::new(),
            outbound: Vec::new(),
            delayed: Vec::new(),
            recorder: Recorder::new(),
            faults: Vec::new(),
        }
    }

    pub fn time(&self) -> Timestamp {
        self.time
    }

    pub fn recorder(&self) -> &Recorder {
        &self.recorder
    }

    pub fn recorder_mut(&mut self) -> &mut Recorder {
        &mut self.recorder
    }

    /// Add an ECU. `listen` lists the frame ids it wants to receive.
    pub fn attach(&mut self, ecu: Box<dyn Ecu>, listen: &[u32]) -> usize {
        let index = self.ecus.len();
        for id in listen {
            self.subscriptions.entry(*id).or_default().push(index);
        }
        self.ecus.push(ecu);
        index
    }

    pub fn subscribe(&mut self, id: u32, ecu_index: usize) {
        self.subscriptions.entry(id).or_default().push(ecu_index);
    }

    pub fn add_fault(&mut self, rule: FaultRule) {
        self.faults.push(rule);
    }

    pub fn set_signal(
        &mut self,
        ecu_index: usize,
        id: u32,
        signal: &str,
        value: SignalValue,
    ) -> Result<(), EcuError> {
        self.ecus[ecu_index].set_signal(id, signal, value)
    }

    /// Inject a frame into the bus as if an external node transmitted it.
    pub fn inject(&mut self, mut frame: CanFrame) {
        frame.ts = self.time;
        self.recorder.frame(frame.clone());
        self.deliver(&frame);
    }

    /// Advance the simulation by one tick.
    ///
    /// Each tick runs `Ecu::update` at the current time, routes the resulting
    /// frames through the fault layer, then advances the clock by `step_us`.
    /// A `run_until(t)` therefore processes ticks at times `0, step, 2·step,
    /// …` strictly below `t`, ending with the clock exactly at `t`.
    pub fn step(&mut self) {
        let mut outbound = std::mem::take(&mut self.outbound);
        for ecu in self.ecus.iter_mut() {
            ecu.update(self.time, &mut outbound);
        }

        for frame in outbound {
            let mut frame = frame;
            frame.ts = self.time;
            self.route(frame);
        }

        // Deliver frames whose delay window has elapsed.
        let mut due: Vec<CanFrame> = Vec::new();
        self.delayed.retain(|(at, f)| {
            if *at <= self.time {
                due.push(f.clone());
                false
            } else {
                true
            }
        });
        for frame in due {
            self.recorder.frame(frame.clone());
            self.deliver(&frame);
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

    /// Frame count per id, ordered by id (deterministic).
    pub fn frame_counts(&self) -> Vec<(u32, usize)> {
        let mut counts: BTreeMap<u32, usize> = BTreeMap::new();
        for f in self.recorder.frames() {
            *counts.entry(f.id).or_default() += 1;
        }
        counts.into_iter().collect()
    }

    fn route(&mut self, mut frame: CanFrame) {
        match self.apply_faults(&frame) {
            Action::Deliver => {
                self.recorder.frame(frame.clone());
                self.deliver(&frame);
            }
            Action::Drop => {
                self.recorder.event(
                    self.time,
                    "bus",
                    format!("dropped frame 0x{:03X}", frame.id),
                );
            }
            Action::Delay(delay) => {
                self.recorder.event(
                    self.time,
                    "bus",
                    format!("delayed frame 0x{:03X} by {delay}us", frame.id),
                );
                frame.ts += delay;
                self.delayed.push((self.time + delay, frame));
            }
            Action::Corrupt { byte, mask } => {
                if let Some(b) = frame.data.get_mut(byte) {
                    *b ^= mask;
                }
                self.recorder.event(
                    self.time,
                    "bus",
                    format!("corrupted byte {byte} of 0x{:03X}", frame.id),
                );
                self.recorder.frame(frame.clone());
                self.deliver(&frame);
            }
        }
    }

    fn apply_faults(&self, frame: &CanFrame) -> Action {
        for rule in &self.faults {
            if !rule.active_at(self.time) {
                continue;
            }
            match rule.fault {
                Fault::DropFrame { id } if id == frame.id => return Action::Drop,
                Fault::DelayFrame { id, delay_us } if id == frame.id => {
                    return Action::Delay(delay_us)
                }
                Fault::CorruptByte { id, byte, mask } if id == frame.id => {
                    return Action::Corrupt { byte, mask }
                }
                _ => {}
            }
        }
        Action::Deliver
    }

    fn deliver(&mut self, frame: &CanFrame) {
        if let Some(indices) = self.subscriptions.get(&frame.id) {
            for &i in indices {
                self.ecus[i].on_message(frame, self.time);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ecu::Ecu;
    use crate::recorder::Record;
    use crate::time::{ms, sec, US_PER_MS};

    /// Transmits one frame every period.
    struct PeriodicEcu {
        name: &'static str,
        id: u32,
        period: Timestamp,
        next: Timestamp,
    }

    impl Ecu for PeriodicEcu {
        fn name(&self) -> &str {
            self.name
        }
        fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
            if time >= self.next {
                out.push(CanFrame::new(self.id, vec![0; 8]).unwrap());
                self.next = time + self.period;
            }
        }
    }

    fn echo_sim() -> (Simulation, usize, usize) {
        let mut sim = Simulation::new(US_PER_MS);
        let tx = sim.attach(
            Box::new(PeriodicEcu {
                name: "tx",
                id: 0x100,
                period: ms(10),
                next: 0,
            }),
            &[],
        );
        // Echo reacts to 0x100 by emitting 0x200 during the next update.
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

    /// Emits one `listen + 0x100` frame per received frame.
    struct EchoOnUpdate {
        listen: u32,
        seen: u32,
        emitted: u32,
    }

    impl Ecu for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }
        fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
            if frame.id == self.listen {
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

    #[test]
    fn routing_delivers_only_subscribed_ids() {
        let (mut sim, _tx, _rx) = echo_sim();
        sim.run_ms(110);
        // 0x100 every 10ms -> 11 transmissions (t=0..100, the t=110 tick is
        // not reached). Each delivered frame is echoed once as 0x200 on the
        // following tick, so all 11 echoes land within the run window.
        assert_eq!(
            sim.recorder()
                .frames()
                .iter()
                .filter(|f| f.id == 0x100)
                .count(),
            11
        );
        assert_eq!(
            sim.recorder()
                .frames()
                .iter()
                .filter(|f| f.id == 0x200)
                .count(),
            11
        );
    }

    #[test]
    fn unsubscribed_ids_are_not_delivered() {
        let mut sim = Simulation::new(US_PER_MS);
        let rx = sim.attach(
            Box::new(EchoOnUpdate {
                listen: 0x100,
                seen: 0,
                emitted: 0,
            }),
            &[0x100],
        );
        let _ = rx;
        // Inject a frame the ECU is not subscribed to.
        sim.inject(CanFrame::new(0x500, vec![0; 8]).unwrap());
        sim.step();
        // It must appear on the bus but not trigger an echo.
        assert!(sim.recorder().has_frame(0x500));
        assert!(!sim.recorder().has_frame(0x600));
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
    fn drop_fault_suppresses_frames_then_recovers() {
        let (mut sim, _, _) = echo_sim();
        sim.add_fault(FaultRule {
            fault: Fault::DropFrame { id: 0x100 },
            start: ms(20),
            duration: Some(ms(20)),
        });
        sim.run_ms(50);
        // Produced at t=0,10,20,30,40. Dropped during [20,40): t=20,30.
        // Delivered (and hence echoed): t=0,10,40.
        assert_eq!(
            sim.recorder()
                .frames()
                .iter()
                .filter(|f| f.id == 0x100)
                .count(),
            3
        );
        let drops = sim
            .recorder()
            .records
            .iter()
            .filter(|r| matches!(r, Record::Event { message, .. } if message.contains("dropped")))
            .count();
        assert_eq!(drops, 2);
    }

    #[test]
    fn delay_fault_holds_frame_back() {
        let mut sim = Simulation::new(US_PER_MS);
        let tx = sim.attach(
            Box::new(PeriodicEcu {
                name: "tx",
                id: 0x100,
                period: ms(10),
                next: 0,
            }),
            &[],
        );
        let _ = tx;
        sim.add_fault(FaultRule {
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
            .frames()
            .iter()
            .filter(|f| f.id == 0x100)
            .map(|f| f.ts)
            .collect();
        // First frame (t=0) delayed to t=5; frames still produced each 10ms.
        // `frames()` is most-recent-first, so the oldest recorded frame is last.
        assert_eq!(*frames.last().unwrap(), ms(5));
        assert_eq!(frames.len(), 3); // t=5 (delayed), t=15 (delayed), t=20 (normal)
    }

    #[test]
    fn injection_is_recorded_and_delivered() {
        let mut sim = Simulation::new(US_PER_MS);
        sim.attach(
            Box::new(EchoOnUpdate {
                listen: 0x200,
                seen: 0,
                emitted: 0,
            }),
            &[0x200],
        );
        sim.inject(CanFrame::new(0x200, vec![1, 0, 0, 0, 0, 0, 0, 0]).unwrap());
        assert!(sim.recorder().has_frame(0x200));
    }

    #[test]
    fn frame_counts_are_ordered_by_id() {
        let (mut sim, _, _) = echo_sim();
        sim.run_ms(10);
        // One 0x100 at t=0, echoed as one 0x200 at t=1.
        assert_eq!(sim.frame_counts(), vec![(0x100, 1), (0x200, 1)]);
    }

    #[test]
    fn sim_seconds_helper() {
        let (mut sim, _, _) = echo_sim();
        sim.run_for(sec(1));
        assert_eq!(sim.time(), sec(1));
    }
}
