//! The test runner: executes a [`TestSpec`] against a [`TestTarget`].
//!
//! Assertions are evaluated by polling the target. In virtual mode each poll
//! advances the simulation by a fixed poll interval, so `within` deadlines are
//! still deterministic; in hardware mode polls are wall-clock waits.

use embrig_core::fault::Fault;
use embrig_core::frame::CanFrame;
use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;
use embrig_net::{MessageDef, UdpDatagram, UdpFault};
use thiserror::Error;

use crate::dsl::{
    parse_duration, ExpectStep, ExpectUdpStep, ExpectedValue, FaultKind, Step, TestSpec,
    UdpFaultKind,
};
use crate::report::{SuiteResult, TestResult};
use crate::target::{TargetError, TestTarget};

/// Errors that abort a whole test (as opposed to failing an assertion).
#[derive(Debug, Error)]
pub enum TestError {
    #[error("{0}")]
    Target(#[from] TargetError),
    #[error("{0}")]
    Dsl(#[from] crate::dsl::DslError),
    #[error("invalid frame: {0}")]
    Frame(#[from] embrig_core::frame::FrameError),
    #[error("{0}")]
    Message(String),
}

impl From<String> for TestError {
    fn from(message: String) -> Self {
        TestError::Message(message)
    }
}

/// Run every test file against a target, collecting one [`SuiteResult`].
///
/// `label` is used as the suite display name in reports. The target is reset
/// before each test so faults, signal overrides and clock state never leak
/// between tests. Assertion failures stop only the failing test.
pub async fn run_suite<T: TestTarget>(
    target: &mut T,
    files: &[std::path::PathBuf],
    label: &str,
) -> Result<SuiteResult, TestError> {
    let start = std::time::Instant::now();
    let mut tests = Vec::with_capacity(files.len());
    for file in files {
        let spec = crate::dsl::load_spec(file)?;
        target.reset()?;
        let result = run_spec(&spec, target).await?;
        tests.push(result);
    }
    Ok(SuiteResult {
        file: label.to_string(),
        duration_us: start.elapsed().as_micros() as u64,
        tests,
    })
}

/// Run one test spec against a target. Assertion failures are recorded inside
/// the returned [`TestResult`]; infrastructure errors abort the test.
pub async fn run_spec<T: TestTarget>(
    spec: &TestSpec,
    target: &mut T,
) -> Result<TestResult, TestError> {
    let timeout = parse_duration(&spec.timeout)?;
    let start = target.elapsed_us();
    let deadline = start.saturating_add(timeout);
    let mut result = TestResult {
        name: spec.name.clone(),
        passed: true,
        steps: spec.steps.len(),
        duration_us: 0,
        failures: Vec::new(),
    };

    for step in &spec.steps {
        let outcome: Result<(), String> = match step {
            Step::Send { id, data } => {
                let frame = CanFrame::new(*id, data.clone())?;
                target.send(frame).await.map_err(|e| e.to_string())
            }
            Step::SetSignal {
                ecu,
                id,
                signal,
                value,
            } => target
                .set_signal(ecu, *id, signal, expected_to_signal(value))
                .map_err(|e| e.to_string()),
            Step::Wait { time } => {
                let duration = parse_duration(time)?;
                target.wait(duration).await.map_err(|e| e.to_string())
            }
            Step::Expect { spec } => evaluate_expect(spec, target, deadline).await,
            Step::Fault {
                kind,
                id,
                delay,
                byte,
                mask,
                start,
                duration,
            } => {
                let fault = fault_from_kind(*kind, *id, delay.as_deref(), *byte, *mask)?;
                let start_us = start
                    .as_deref()
                    .map(parse_duration)
                    .transpose()?
                    .or(Some(target.elapsed_us()));
                let duration_us = duration.as_deref().map(parse_duration).transpose()?;
                target
                    .add_fault(fault, start_us, duration_us)
                    .map_err(|e| e.to_string())
            }
            Step::SendUdp { message, fields } => {
                let netmap = target.netmap().ok_or_else(|| {
                    "send_udp requires a UDP target (interface type `udp`)".to_string()
                })?;
                let message_def = netmap
                    .message(message)
                    .ok_or_else(|| format!("no message `{message}` in netmap"))?;
                let values: Vec<(&str, SignalValue)> = fields
                    .iter()
                    .map(|(k, v)| (k.as_str(), expected_to_signal(v)))
                    .collect();
                let payload = message_def
                    .encode_fields(&values)
                    .map_err(|e| format!("cannot encode `{message}`: {e}"))?;
                let src = target.udp_host().map_err(|e| e.to_string())?;
                target
                    .send_udp(UdpDatagram::new(src, message_def.dst, payload))
                    .await
                    .map_err(|e| e.to_string())
            }
            Step::SetField {
                ecu,
                message,
                field,
                value,
            } => target
                .set_field(ecu, message, field, expected_to_signal(value))
                .map_err(|e| e.to_string()),
            Step::ExpectUdp { spec } => evaluate_udp_expect(spec, target, deadline).await,
            Step::FaultUdp {
                kind,
                message,
                delay,
                byte,
                mask,
                start,
                duration,
            } => {
                let netmap = target.netmap().ok_or_else(|| {
                    "fault_udp requires a UDP target (interface type `udp`)".to_string()
                })?;
                let dst = netmap
                    .message_dst(message)
                    .ok_or_else(|| format!("no message `{message}` in netmap"))?;
                let fault = udp_fault_from_kind(*kind, dst, delay.as_deref(), *byte, *mask)?;
                let start_us = start
                    .as_deref()
                    .map(parse_duration)
                    .transpose()?
                    .or(Some(target.elapsed_us()));
                let duration_us = duration.as_deref().map(parse_duration).transpose()?;
                target
                    .add_fault_udp(fault, start_us, duration_us)
                    .map_err(|e| e.to_string())
            }
        };

        match outcome {
            Ok(()) => {
                // The whole test has a time budget; overrunning it fails the
                // test so a hung test never passes silently.
                if target.elapsed_us() > deadline {
                    result.passed = false;
                    result
                        .failures
                        .push(format!("test exceeded its timeout ({})", spec.timeout));
                    break;
                }
            }
            Err(message) => {
                result.passed = false;
                result.failures.push(message);
                break;
            }
        }
    }

    result.duration_us = target.elapsed_us() - start;
    Ok(result)
}

/// Build a [`Fault`] from the YAML fault step fields.
fn fault_from_kind(
    kind: FaultKind,
    id: u32,
    delay: Option<&str>,
    byte: Option<usize>,
    mask: Option<u8>,
) -> Result<Fault, TestError> {
    Ok(match kind {
        FaultKind::Drop => Fault::DropFrame { id },
        FaultKind::Delay => Fault::DelayFrame {
            id,
            delay_us: parse_duration(delay.expect("validated by load_spec"))?,
        },
        FaultKind::Corrupt => Fault::CorruptByte {
            id,
            byte: byte.expect("validated by load_spec"),
            mask: mask.expect("validated by load_spec"),
        },
    })
}

fn expected_to_signal(value: &ExpectedValue) -> SignalValue {
    match value {
        ExpectedValue::Num(v) => SignalValue::Num(*v),
        ExpectedValue::Bool(b) => SignalValue::Num(if *b { 1.0 } else { 0.0 }),
        ExpectedValue::Str(s) => SignalValue::Str(s.clone()),
    }
}

/// Poll an assertion until it holds or its `within` deadline passes. Polling
/// is capped by the whole-test `test_deadline` so a huge `within` cannot run
/// a test past its budget.
async fn evaluate_expect<T: TestTarget>(
    spec: &ExpectStep,
    target: &mut T,
    test_deadline: Timestamp,
) -> Result<(), String> {
    let within = match &spec.within {
        Some(w) => parse_duration(w).map_err(|e| e.to_string())?,
        None => 0,
    };
    let expect_deadline = target.elapsed_us().saturating_add(within);
    let deadline = expect_deadline.min(test_deadline);

    loop {
        let frame = target.poll(spec.id).await.map_err(|e| e.to_string())?;

        if spec.effective_present() == Some(false) && frame.is_some() {
            return Err(format!(
                "expected no frame 0x{:03X} but one was observed",
                spec.id
            ));
        }

        match check(spec, frame.as_ref(), target.network()) {
            Ok(()) => return Ok(()),
            Err(message) => {
                if within == 0 || target.elapsed_us() >= deadline {
                    return Err(message);
                }
            }
        }
    }
}

/// Evaluate an assertion against the latest observed frame (or its absence).
fn check(spec: &ExpectStep, frame: Option<&CanFrame>, network: &Network) -> Result<(), String> {
    if let Some(present) = spec.effective_present() {
        return match (present, frame) {
            (true, Some(_)) => Ok(()),
            (false, None) => Ok(()),
            (true, None) => Err(format!("expected a frame 0x{:03X} to be present", spec.id)),
            (false, Some(_)) => unreachable!("handled before check"),
        };
    }

    let frame = frame.ok_or_else(|| format!("no frame 0x{:03X} observed", spec.id))?;
    let message = network
        .message(spec.id)
        .ok_or_else(|| format!("no message with id 0x{:03X} in DBC", spec.id))?;
    let signals = message
        .decode_signals(&frame.data)
        .map_err(|e| format!("cannot decode 0x{:03X}: {e}", spec.id))?;
    let signal_name = spec.signal.as_deref().unwrap_or_default();
    let signal = signals
        .iter()
        .find(|s| s.name == signal_name)
        .ok_or_else(|| format!("unknown signal `{signal_name}` on 0x{:03X}", spec.id))?;

    if let Some(expected) = &spec.equals {
        let pass = match expected {
            ExpectedValue::Num(v) => (signal.value - v).abs() < 1e-6,
            ExpectedValue::Bool(b) => (signal.value > 0.5) == *b,
            ExpectedValue::Str(s) => signal.symbol.as_deref() == Some(s.as_str()),
        };
        if !pass {
            let got = signal
                .symbol
                .as_deref()
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("{}", signal.value));
            return Err(format!(
                "0x{:03X}.{} expected {}, got {}",
                spec.id,
                signal_name,
                expected.describe(),
                got
            ));
        }
    }
    if let Some(v) = spec.greater_than {
        if signal.value <= v {
            return Err(format!(
                "0x{:03X}.{} expected > {v}, got {}",
                spec.id, signal_name, signal.value
            ));
        }
    }
    if let Some(v) = spec.less_than {
        if signal.value >= v {
            return Err(format!(
                "0x{:03X}.{} expected < {v}, got {}",
                spec.id, signal_name, signal.value
            ));
        }
    }
    Ok(())
}

/// Build a [`UdpFault`] from the YAML fault_udp step fields.
fn udp_fault_from_kind(
    kind: UdpFaultKind,
    dst: std::net::SocketAddr,
    delay: Option<&str>,
    byte: Option<usize>,
    mask: Option<u8>,
) -> Result<UdpFault, TestError> {
    Ok(match kind {
        UdpFaultKind::Drop => UdpFault::Drop { dst },
        UdpFaultKind::Delay => UdpFault::Delay {
            dst,
            delay_us: parse_duration(delay.expect("validated by load_spec"))?,
        },
        UdpFaultKind::Corrupt => UdpFault::CorruptByte {
            dst,
            byte: byte.expect("validated by load_spec"),
            mask: mask.expect("validated by load_spec"),
        },
    })
}

/// Poll a UDP assertion until it holds or its `within` deadline passes. The
/// message's destination endpoint (from the netmap) identifies the traffic.
async fn evaluate_udp_expect<T: TestTarget>(
    spec: &ExpectUdpStep,
    target: &mut T,
    test_deadline: Timestamp,
) -> Result<(), String> {
    let message = target
        .netmap()
        .ok_or_else(|| "expect_udp requires a UDP target (interface type `udp`)".to_string())?
        .message(&spec.message)
        .cloned()
        .ok_or_else(|| format!("no message `{}` in netmap", spec.message))?;
    let dst = message.dst;

    let within = match &spec.within {
        Some(w) => parse_duration(w).map_err(|e| e.to_string())?,
        None => 0,
    };
    let expect_deadline = target.elapsed_us().saturating_add(within);
    let deadline = expect_deadline.min(test_deadline);

    loop {
        let dg = target.poll_udp(dst).await.map_err(|e| e.to_string())?;

        if spec.effective_present() == Some(false) && dg.is_some() {
            return Err(format!(
                "expected no message `{}` but one was observed",
                spec.message
            ));
        }

        match check_udp(spec, dg.as_ref(), &message) {
            Ok(()) => return Ok(()),
            Err(message) => {
                if within == 0 || target.elapsed_us() >= deadline {
                    return Err(message);
                }
            }
        }
    }
}

/// Evaluate a UDP assertion against the latest observed datagram (or its
/// absence).
fn check_udp(
    spec: &ExpectUdpStep,
    dg: Option<&UdpDatagram>,
    message: &MessageDef,
) -> Result<(), String> {
    if let Some(present) = spec.effective_present() {
        return match (present, dg) {
            (true, Some(_)) => Ok(()),
            (false, None) => Ok(()),
            (true, None) => Err(format!("expected message `{}` to be present", spec.message)),
            (false, Some(_)) => unreachable!("handled before check_udp"),
        };
    }

    let dg = dg.ok_or_else(|| format!("no message `{}` observed", spec.message))?;
    let field = spec.field.as_deref().unwrap_or_default();
    let decoded = message
        .decode_field(&dg.payload, field)
        .map_err(|e| format!("cannot decode `{}`: {e}", spec.message))?;

    if let Some(expected) = &spec.equals {
        let pass = match expected {
            ExpectedValue::Num(v) => (decoded.value - v).abs() < 1e-6,
            ExpectedValue::Bool(b) => (decoded.value > 0.5) == *b,
            ExpectedValue::Str(s) => decoded.symbol.as_deref() == Some(s.as_str()),
        };
        if !pass {
            let got = decoded
                .symbol
                .as_deref()
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("{}", decoded.value));
            return Err(format!(
                "`{}`.{} expected {}, got {}",
                spec.message,
                field,
                expected.describe(),
                got
            ));
        }
    }
    if let Some(v) = spec.greater_than {
        if decoded.value <= v {
            return Err(format!(
                "`{}`.{} expected > {v}, got {}",
                spec.message, field, decoded.value
            ));
        }
    }
    if let Some(v) = spec.less_than {
        if decoded.value >= v {
            return Err(format!(
                "`{}`.{} expected < {v}, got {}",
                spec.message, field, decoded.value
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::target::{TargetError, POLL_US};
    use embrig_core::frame::CanFrame;
    use std::collections::BTreeMap;
    use std::net::SocketAddr;

    const DBC: &str = r#"VERSION ""

NS_ :

BS_:

BU_: engine

BO_ 256 BatteryStatus: 8 engine
 SG_ voltage : 0|16@1+ (0.1,0) [0|600] "V" engine
 SG_ state : 16|4@1+ (1,0) [0|4] "" engine

BO_ 544 MotorEnable: 8 engine
 SG_ motor_enable : 0|1@1+ (1,0) [0|1] "" engine

VAL_ 256 state 0 "OFF" 1 "INIT" 2 "READY" 3 "CHARGING" 4 "FAULT" ;
"#;

    fn network() -> Network {
        embrig_dbc::parse(DBC).unwrap()
    }

    fn spec(id: u32) -> ExpectStep {
        ExpectStep {
            id,
            ..ExpectStep::default()
        }
    }

    fn frame(id: u32, data: Vec<u8>) -> CanFrame {
        CanFrame::new(id, data).unwrap()
    }

    fn battery(voltage: f64, state: &str) -> CanFrame {
        let n = network();
        let message = n.message(0x100).unwrap();
        let raw = message
            .encode_signals(&[
                ("voltage", voltage),
                (
                    "state",
                    message.physical_for_symbol("state", state).unwrap(),
                ),
            ])
            .unwrap();
        frame(0x100, raw)
    }

    fn motor(enabled: bool) -> CanFrame {
        let n = network();
        let message = n.message(0x220).unwrap();
        let raw = message
            .encode_signals(&[("motor_enable", if enabled { 1.0 } else { 0.0 })])
            .unwrap();
        frame(0x220, raw)
    }

    #[test]
    fn check_present() {
        let n = network();
        let present = ExpectStep {
            present: Some(true),
            ..spec(0x100)
        };
        assert!(check(&present, Some(&battery(400.0, "READY")), &n).is_ok());
        let err = check(&present, None, &n).unwrap_err();
        assert!(err.contains("present"), "got: {err}");

        let absent = ExpectStep {
            present: Some(false),
            ..spec(0x100)
        };
        assert!(check(&absent, None, &n).is_ok());
    }

    #[test]
    fn check_equals_numeric() {
        let n = network();
        let frame = battery(400.0, "READY");
        let pass = ExpectStep {
            signal: Some("voltage".into()),
            equals: Some(ExpectedValue::Num(400.0)),
            ..spec(0x100)
        };
        assert!(check(&pass, Some(&frame), &n).is_ok());
        let fail = ExpectStep {
            signal: Some("voltage".into()),
            equals: Some(ExpectedValue::Num(401.0)),
            ..spec(0x100)
        };
        let err = check(&fail, Some(&frame), &n).unwrap_err();
        assert!(err.contains("expected 401, got 400"), "got: {err}");
    }

    #[test]
    fn check_equals_bool() {
        let n = network();
        let on = motor(true);
        let pass = ExpectStep {
            signal: Some("motor_enable".into()),
            equals: Some(ExpectedValue::Bool(true)),
            ..spec(0x220)
        };
        assert!(check(&pass, Some(&on), &n).is_ok());
        let fail = ExpectStep {
            signal: Some("motor_enable".into()),
            equals: Some(ExpectedValue::Bool(false)),
            ..spec(0x220)
        };
        assert!(check(&fail, Some(&on), &n).is_err());
    }

    #[test]
    fn check_equals_symbol() {
        let n = network();
        let frame = battery(400.0, "READY");
        let pass = ExpectStep {
            signal: Some("state".into()),
            equals: Some(ExpectedValue::Str("READY".into())),
            ..spec(0x100)
        };
        assert!(check(&pass, Some(&frame), &n).is_ok());
        let fail = ExpectStep {
            signal: Some("state".into()),
            equals: Some(ExpectedValue::Str("FAULT".into())),
            ..spec(0x100)
        };
        assert!(check(&fail, Some(&frame), &n).is_err());
    }

    #[test]
    fn check_comparisons() {
        let n = network();
        let frame = battery(400.0, "READY");
        let gt_pass = ExpectStep {
            signal: Some("voltage".into()),
            greater_than: Some(350.0),
            ..spec(0x100)
        };
        assert!(check(&gt_pass, Some(&frame), &n).is_ok());
        let gt_fail = ExpectStep {
            signal: Some("voltage".into()),
            greater_than: Some(450.0),
            ..spec(0x100)
        };
        let err = check(&gt_fail, Some(&frame), &n).unwrap_err();
        assert!(err.contains("expected > 450, got 400"), "got: {err}");
        let lt_pass = ExpectStep {
            signal: Some("voltage".into()),
            less_than: Some(450.0),
            ..spec(0x100)
        };
        assert!(check(&lt_pass, Some(&frame), &n).is_ok());
    }

    #[test]
    fn check_unknown_signal_and_frame() {
        let n = network();
        let battery_frame = battery(400.0, "READY");
        let unknown = ExpectStep {
            signal: Some("nope".into()),
            equals: Some(ExpectedValue::Num(1.0)),
            ..spec(0x100)
        };
        assert!(check(&unknown, Some(&battery_frame), &n).is_err());

        let missing = ExpectStep {
            signal: Some("voltage".into()),
            equals: Some(ExpectedValue::Num(400.0)),
            ..spec(0x100)
        };
        let err = check(&missing, None, &n).unwrap_err();
        assert!(err.contains("no frame"), "got: {err}");

        let not_in_dbc = ExpectStep {
            signal: Some("voltage".into()),
            equals: Some(ExpectedValue::Num(400.0)),
            ..spec(0x999)
        };
        assert!(check(&not_in_dbc, Some(&frame(0x999, vec![0; 8])), &n).is_err());
    }

    /// Minimal deterministic target: frames are returned by `poll` once their
    /// timestamp has been reached; `wait`/`poll` advance `elapsed_us`.
    struct MockTarget {
        network: Network,
        time: Timestamp,
        visible: Vec<CanFrame>,
        later: Vec<(Timestamp, CanFrame)>,
    }

    impl MockTarget {
        fn new() -> Self {
            Self {
                network: network(),
                time: 0,
                visible: Vec::new(),
                later: Vec::new(),
            }
        }

        fn push_now(&mut self, f: CanFrame) {
            self.visible.push(f);
        }

        fn push_at(&mut self, f: CanFrame, at_us: Timestamp) {
            self.later.push((at_us, f));
        }

        fn flush(&mut self) {
            let time = self.time;
            let mut i = 0;
            while i < self.later.len() {
                if self.later[i].0 <= time {
                    let (_, f) = self.later.remove(i);
                    self.visible.push(f);
                } else {
                    i += 1;
                }
            }
        }
    }

    impl TestTarget for MockTarget {
        fn network(&self) -> &Network {
            &self.network
        }

        fn elapsed_us(&self) -> Timestamp {
            self.time
        }

        fn reset(&mut self) -> Result<(), TargetError> {
            self.time = 0;
            self.visible.clear();
            self.later.clear();
            Ok(())
        }

        fn set_signal(
            &mut self,
            _ecu: &str,
            _id: u32,
            _signal: &str,
            _value: SignalValue,
        ) -> Result<(), TargetError> {
            Err(TargetError::UnsupportedOnHardware("mock".into()))
        }

        fn add_fault(
            &mut self,
            _fault: Fault,
            _start: Option<Timestamp>,
            _duration: Option<Timestamp>,
        ) -> Result<(), TargetError> {
            Err(TargetError::UnsupportedOnHardware("mock".into()))
        }

        async fn send(&mut self, f: CanFrame) -> Result<(), TargetError> {
            self.push_now(f);
            Ok(())
        }

        async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
            self.time += duration;
            self.flush();
            Ok(())
        }

        async fn poll(&mut self, id: u32) -> Result<Option<CanFrame>, TargetError> {
            self.time += POLL_US;
            self.flush();
            Ok(self.visible.iter().rev().find(|f| f.id == id).cloned())
        }
    }

    #[tokio::test]
    async fn expect_present_polls_until_frame_arrives() {
        let mut target = MockTarget::new();
        target.push_at(battery(400.0, "READY"), 25_000);
        let mut s = spec(0x100);
        s.present = Some(true);
        s.within = Some("50ms".into());
        evaluate_expect(&s, &mut target, 1_000_000).await.unwrap();
        assert!(target.elapsed_us() >= 25_000, "polled past arrival time");
    }

    #[tokio::test]
    async fn expect_present_times_out() {
        let mut target = MockTarget::new();
        let mut s = spec(0x100);
        s.present = Some(true);
        s.within = Some("50ms".into());
        let err = evaluate_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("present"), "got: {err}");
        assert_eq!(target.elapsed_us(), 50_000, "polled until the deadline");
    }

    #[tokio::test]
    async fn expect_absent_rejects_seen_frame() {
        let mut target = MockTarget::new();
        target.push_now(battery(400.0, "READY"));
        let mut s = spec(0x100);
        s.present = Some(false);
        s.within = Some("10ms".into());
        let err = evaluate_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("expected no frame"), "got: {err}");
    }

    #[tokio::test]
    async fn expect_absent_passes_without_frame() {
        let mut target = MockTarget::new();
        let mut s = spec(0x100);
        s.present = Some(false);
        evaluate_expect(&s, &mut target, 1_000_000).await.unwrap();
    }

    #[tokio::test]
    async fn expect_without_within_checks_once() {
        let mut target = MockTarget::new();
        target.push_now(battery(400.0, "READY"));
        let mut pass = spec(0x100);
        pass.present = Some(true);
        evaluate_expect(&pass, &mut target, 1_000_000)
            .await
            .unwrap();
        assert_eq!(target.elapsed_us(), POLL_US, "single poll, no retry");

        let mut target = MockTarget::new();
        let mut fail = spec(0x100);
        fail.present = Some(true);
        let err = evaluate_expect(&fail, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("present"), "got: {err}");
        assert_eq!(
            target.elapsed_us(),
            POLL_US,
            "checked once, no deadline wait"
        );
    }

    #[tokio::test]
    async fn expect_equals_retries_until_deadline() {
        let mut target = MockTarget::new();
        target.push_now(battery(400.0, "READY"));
        let mut s = spec(0x100);
        s.signal = Some("voltage".into());
        s.equals = Some(ExpectedValue::Num(500.0));
        s.within = Some("50ms".into());
        let err = evaluate_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("expected 500, got 400"), "got: {err}");
        assert_eq!(target.elapsed_us(), 50_000);
    }

    #[tokio::test]
    async fn expect_absent_alias_rejects_seen_frame() {
        let mut target = MockTarget::new();
        target.push_now(battery(400.0, "READY"));
        let mut s = spec(0x100);
        s.absent = Some(true);
        let err = evaluate_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("expected no frame"), "got: {err}");
    }

    #[tokio::test]
    async fn run_spec_fails_when_test_exceeds_timeout() {
        let mut target = MockTarget::new();
        let spec = TestSpec {
            name: "slow".into(),
            timeout: "10ms".into(),
            steps: vec![Step::Wait {
                time: "100ms".into(),
            }],
        };
        let result = run_spec(&spec, &mut target).await.unwrap();
        assert!(!result.passed);
        assert!(
            result.failures[0].contains("timeout"),
            "got: {:?}",
            result.failures
        );
    }

    #[tokio::test]
    async fn run_spec_passes_within_budget() {
        let mut target = MockTarget::new();
        let spec = TestSpec {
            name: "quick".into(),
            timeout: "1s".into(),
            steps: vec![Step::Wait {
                time: "50ms".into(),
            }],
        };
        let result = run_spec(&spec, &mut target).await.unwrap();
        assert!(result.passed, "got: {:?}", result.failures);
    }

    #[tokio::test]
    async fn run_spec_expect_capped_by_test_deadline() {
        // A huge `within` cannot push a test past its overall budget: the
        // poll loop stops at the test deadline, not the `within` deadline.
        let mut target = MockTarget::new();
        let spec = TestSpec {
            name: "capped".into(),
            timeout: "30ms".into(),
            steps: vec![Step::Expect {
                spec: ExpectStep {
                    id: 0x100,
                    present: Some(true),
                    within: Some("10s".into()),
                    ..ExpectStep::default()
                },
            }],
        };
        let result = run_spec(&spec, &mut target).await.unwrap();
        assert!(!result.passed);
        assert_eq!(target.elapsed_us(), 30_000);
    }

    // ---- UDP assertion helpers ----

    use embrig_net::netmap::{FieldDef, FieldType, Netmap};

    fn udp_netmap() -> Netmap {
        let mut netmap = Netmap::new();
        netmap.messages.insert(
            "MotionState".to_string(),
            embrig_net::MessageDef {
                dst: "127.0.0.1:5000".parse().unwrap(),
                length: 8,
                fields: BTreeMap::from([
                    (
                        "speed".to_string(),
                        FieldDef {
                            offset: 0,
                            ty: FieldType::F32le,
                            factor: 1.0,
                            shift: 0.0,
                            values: BTreeMap::new(),
                        },
                    ),
                    (
                        "state".to_string(),
                        FieldDef {
                            offset: 4,
                            ty: FieldType::U8,
                            factor: 1.0,
                            shift: 0.0,
                            values: BTreeMap::from([(0, "STOPPED".into()), (1, "DRIVING".into())]),
                        },
                    ),
                ]),
            },
        );
        netmap
    }

    fn udp_message() -> MessageDef {
        udp_netmap().message("MotionState").unwrap().clone()
    }

    fn udp_spec(field: &str) -> ExpectUdpStep {
        ExpectUdpStep {
            message: "MotionState".into(),
            field: Some(field.into()),
            ..ExpectUdpStep::default()
        }
    }

    fn udp_datagram(speed: f32, state: u8) -> UdpDatagram {
        let message = udp_message();
        let payload = message
            .encode_fields(&[
                ("speed", SignalValue::Num(speed as f64)),
                ("state", SignalValue::Num(state as f64)),
            ])
            .unwrap();
        UdpDatagram::new("127.0.0.1:6000".parse().unwrap(), message.dst, payload)
    }

    #[test]
    fn check_udp_equals_numeric_and_symbol() {
        let dg = udp_datagram(1.5, 1);
        let pass = ExpectUdpStep {
            equals: Some(ExpectedValue::Num(1.5)),
            ..udp_spec("speed")
        };
        assert!(check_udp(&pass, Some(&dg), &udp_message()).is_ok());
        let fail = ExpectUdpStep {
            equals: Some(ExpectedValue::Num(2.0)),
            ..udp_spec("speed")
        };
        let err = check_udp(&fail, Some(&dg), &udp_message()).unwrap_err();
        assert!(err.contains("expected 2, got 1.5"), "got: {err}");

        let symbol = ExpectUdpStep {
            equals: Some(ExpectedValue::Str("DRIVING".into())),
            ..udp_spec("state")
        };
        assert!(check_udp(&symbol, Some(&dg), &udp_message()).is_ok());
        let symbol_fail = ExpectUdpStep {
            equals: Some(ExpectedValue::Str("STOPPED".into())),
            ..udp_spec("state")
        };
        assert!(check_udp(&symbol_fail, Some(&dg), &udp_message()).is_err());
    }

    #[test]
    fn check_udp_comparisons_and_present() {
        let dg = udp_datagram(1.5, 0);
        let gt = ExpectUdpStep {
            greater_than: Some(1.0),
            ..udp_spec("speed")
        };
        assert!(check_udp(&gt, Some(&dg), &udp_message()).is_ok());
        let lt = ExpectUdpStep {
            less_than: Some(5.0),
            ..udp_spec("speed")
        };
        assert!(check_udp(&lt, Some(&dg), &udp_message()).is_ok());
        let present = ExpectUdpStep {
            present: Some(true),
            ..ExpectUdpStep::default()
        };
        assert!(check_udp(&present, Some(&dg), &udp_message()).is_ok());
        assert!(check_udp(&present, None, &udp_message()).is_err());
    }

    #[test]
    fn check_udp_unknown_field() {
        let dg = udp_datagram(1.5, 0);
        let unknown = ExpectUdpStep {
            equals: Some(ExpectedValue::Num(1.0)),
            ..udp_spec("nope")
        };
        let err = check_udp(&unknown, Some(&dg), &udp_message()).unwrap_err();
        assert!(err.contains("cannot decode"), "got: {err}");
    }

    /// A minimal UDP target: `poll_udp` advances time by a poll interval and
    /// returns the most recent datagram for `dst`.
    struct MockUdpTarget {
        time: Timestamp,
        netmap: Netmap,
        visible: Vec<UdpDatagram>,
        later: Vec<(Timestamp, UdpDatagram)>,
    }

    impl MockUdpTarget {
        fn new() -> Self {
            Self {
                time: 0,
                netmap: udp_netmap(),
                visible: Vec::new(),
                later: Vec::new(),
            }
        }

        fn push_at(&mut self, dg: UdpDatagram, at_us: Timestamp) {
            self.later.push((at_us, dg));
        }

        fn flush(&mut self) {
            let time = self.time;
            let mut i = 0;
            while i < self.later.len() {
                if self.later[i].0 <= time {
                    let (_, dg) = self.later.remove(i);
                    self.visible.push(dg);
                } else {
                    i += 1;
                }
            }
        }
    }

    impl TestTarget for MockUdpTarget {
        fn network(&self) -> &Network {
            unimplemented!("no DBC network for a UDP-only mock")
        }
        fn elapsed_us(&self) -> Timestamp {
            self.time
        }
        fn reset(&mut self) -> Result<(), TargetError> {
            self.time = 0;
            self.visible.clear();
            self.later.clear();
            Ok(())
        }
        fn set_signal(
            &mut self,
            _ecu: &str,
            _id: u32,
            _signal: &str,
            _value: SignalValue,
        ) -> Result<(), TargetError> {
            Err(TargetError::UnsupportedOnTarget("no CAN".into()))
        }
        fn add_fault(
            &mut self,
            _fault: Fault,
            _start: Option<Timestamp>,
            _duration: Option<Timestamp>,
        ) -> Result<(), TargetError> {
            Err(TargetError::UnsupportedOnTarget("no CAN".into()))
        }
        async fn send(&mut self, _frame: CanFrame) -> Result<(), TargetError> {
            Err(TargetError::UnsupportedOnTarget("no CAN".into()))
        }
        async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
            self.time += duration;
            self.flush();
            Ok(())
        }
        async fn poll(&mut self, _id: u32) -> Result<Option<CanFrame>, TargetError> {
            Err(TargetError::UnsupportedOnTarget("no CAN".into()))
        }
        fn netmap(&self) -> Option<&Netmap> {
            Some(&self.netmap)
        }
        fn udp_host(&self) -> Result<SocketAddr, TargetError> {
            Ok("127.0.0.1:5000".parse().unwrap())
        }
        async fn send_udp(&mut self, dg: UdpDatagram) -> Result<(), TargetError> {
            self.visible.push(dg);
            Ok(())
        }
        async fn poll_udp(&mut self, dst: SocketAddr) -> Result<Option<UdpDatagram>, TargetError> {
            self.time += POLL_US;
            self.flush();
            Ok(self.visible.iter().rev().find(|d| d.dst == dst).cloned())
        }
    }

    #[tokio::test]
    async fn expect_udp_polls_until_message_arrives() {
        let mut target = MockUdpTarget::new();
        target.push_at(udp_datagram(1.5, 1), 25_000);
        let mut s = udp_spec("speed");
        s.equals = Some(ExpectedValue::Num(1.5));
        s.within = Some("50ms".into());
        evaluate_udp_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap();
        assert!(target.elapsed_us() >= 25_000);
    }

    #[tokio::test]
    async fn expect_udp_times_out() {
        let mut target = MockUdpTarget::new();
        let mut s = udp_spec("speed");
        s.equals = Some(ExpectedValue::Num(9.0));
        s.within = Some("50ms".into());
        let err = evaluate_udp_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(
            err.contains("no message `MotionState` observed"),
            "got: {err}"
        );
        assert_eq!(target.elapsed_us(), 50_000);
    }

    #[tokio::test]
    async fn expect_udp_absent_rejects_seen_message() {
        let mut target = MockUdpTarget::new();
        target.visible.push(udp_datagram(1.5, 1));
        let mut s = ExpectUdpStep {
            absent: Some(true),
            within: Some("10ms".into()),
            ..ExpectUdpStep::default()
        };
        s.message = "MotionState".into();
        let err = evaluate_udp_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("expected no message"), "got: {err}");
    }

    #[tokio::test]
    async fn run_spec_dispatches_udp_steps() {
        let mut target = MockUdpTarget::new();
        let spec = TestSpec {
            name: "udp".into(),
            timeout: "1s".into(),
            steps: vec![
                Step::SendUdp {
                    message: "MotionState".into(),
                    fields: BTreeMap::from([("speed".into(), ExpectedValue::Num(3.0))]),
                },
                Step::ExpectUdp {
                    spec: ExpectUdpStep {
                        message: "MotionState".into(),
                        field: Some("speed".into()),
                        equals: Some(ExpectedValue::Num(3.0)),
                        within: Some("1s".into()),
                        ..ExpectUdpStep::default()
                    },
                },
            ],
        };
        let result = run_spec(&spec, &mut target).await.unwrap();
        assert!(result.passed, "got: {:?}", result.failures);
    }
}
