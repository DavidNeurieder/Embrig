//! The test runner: executes a [`TestSpec`] against a [`TestTarget`].
//!
//! Assertions are evaluated by polling the target. In virtual mode each poll
//! advances the simulation by a fixed poll interval, so `within` deadlines are
//! still deterministic; in hardware mode polls are wall-clock waits.

use std::future::Future;
use std::pin::Pin;

use embrig_core::fault::Fault;
use embrig_core::frame::CanFrame;
use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;
use embrig_net::{UdpDatagram, UdpFault};
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
pub async fn run_suite<T: TestTarget + ?Sized>(
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

/// The bundled loopback smoke test: send one frame and expect it back on the
/// same bus (own-message reception). Mirrors `scripts/loopback.yaml`.
pub const LOOPBACK_YAML: &str = "name: bus_loopback\ntimeout: 5s\nsteps:\n  - send: { id: 0x7FF, data: [1, 2, 3, 4, 5, 6, 7, 8] }\n  - expect: { id: 0x7FF, present: true, within: 500ms }\n";

/// Run the bundled loopback smoke test against a target.
///
/// Used by the CLI `--check` flag to prove the interface round trip before
/// running a real suite. The target is reset first.
pub async fn run_loopback<T: TestTarget + ?Sized>(target: &mut T) -> Result<TestResult, TestError> {
    let spec = crate::dsl::load_spec_str(LOOPBACK_YAML)
        .expect("the bundled loopback spec must parse and validate");
    target.reset()?;
    run_spec(&spec, target).await
}

/// Run one test spec against a target. Assertion failures are recorded inside
/// the returned [`TestResult`]; infrastructure errors abort the test.
pub async fn run_spec<T: TestTarget + ?Sized>(
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
                    .add_can_fault(fault, start_us, duration_us)
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
                let src = target.host().map_err(|e| e.to_string())?;
                target
                    .send_msg(UdpDatagram::new(src, message_def.dst, payload))
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
                    .add_netmap_fault(fault, start_us, duration_us)
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

/// A generic assertion shared by CAN and netmap (UDP/TCP) expect steps. Built
/// from an [`ExpectStep`] or [`ExpectUdpStep`] at the call site.
struct Assertion {
    /// Kind word used in messages: `frame` or `message`.
    kind: &'static str,
    /// Display label, e.g. `0x100` or `` `MotionState` ``.
    label: String,
    /// Field/signal display name for value assertions.
    field: String,
    present: Option<bool>,
    equals: Option<ExpectedValue>,
    greater_than: Option<f64>,
    less_than: Option<f64>,
}

/// A decoded field/signal value observed on the target.
#[derive(Debug)]
struct DecodedValue {
    value: f64,
    symbol: Option<String>,
}

/// What one poll observed for the asserted message (or its absence).
#[derive(Debug)]
struct Observed {
    value: Option<DecodedValue>,
}

/// Evaluate an assertion against the latest observed message (or its absence).
fn check_assertion(spec: &Assertion, obs: Option<&Observed>) -> Result<(), String> {
    if let Some(present) = spec.present {
        return match (present, obs) {
            (true, Some(_)) => Ok(()),
            (false, None) => Ok(()),
            (true, None) => Err(format!(
                "expected a {} {} to be present",
                spec.kind, spec.label
            )),
            (false, Some(_)) => unreachable!("absent-with-message is handled by the poll loop"),
        };
    }

    let value = obs
        .and_then(|o| o.value.as_ref())
        .ok_or_else(|| format!("no {} {} observed", spec.kind, spec.label))?;

    if let Some(expected) = &spec.equals {
        let pass = match expected {
            ExpectedValue::Num(v) => (value.value - v).abs() < 1e-6,
            ExpectedValue::Bool(b) => (value.value > 0.5) == *b,
            ExpectedValue::Str(s) => value.symbol.as_deref() == Some(s.as_str()),
        };
        if !pass {
            let got = value
                .symbol
                .as_deref()
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("{}", value.value));
            return Err(format!(
                "{}.{} expected {}, got {}",
                spec.label,
                spec.field,
                expected.describe(),
                got
            ));
        }
    }
    if let Some(v) = spec.greater_than {
        if value.value <= v {
            return Err(format!(
                "{}.{} expected > {v}, got {}",
                spec.label, spec.field, value.value
            ));
        }
    }
    if let Some(v) = spec.less_than {
        if value.value >= v {
            return Err(format!(
                "{}.{} expected < {v}, got {}",
                spec.label, spec.field, value.value
            ));
        }
    }
    Ok(())
}

/// Poll an assertion until it holds or its `within` deadline passes. Polling
/// is capped by the whole-test `test_deadline` so a huge `within` cannot run
/// a test past its budget. `poll` returns what one poll observed (or its
/// absence) and is expected to advance time / sleep like the target's poll
/// methods. An `absent` assertion fails on the first poll that sees a message.
async fn poll_until<T, F>(
    spec: &Assertion,
    within: Option<&str>,
    test_deadline: Timestamp,
    target: &mut T,
    mut poll: F,
) -> Result<(), String>
where
    T: TestTarget + ?Sized,
    F: for<'a> FnMut(
        &'a mut T,
    ) -> Pin<Box<dyn Future<Output = Result<Option<Observed>, String>> + 'a>>,
{
    let within_us = match within {
        Some(w) => parse_duration(w).map_err(|e| e.to_string())?,
        None => 0,
    };
    let expect_deadline = target.elapsed_us().saturating_add(within_us);
    let deadline = expect_deadline.min(test_deadline);

    loop {
        let obs = poll(target).await?;

        if spec.present == Some(false) && obs.is_some() {
            return Err(format!(
                "expected no {} {} but one was observed",
                spec.kind, spec.label
            ));
        }

        match check_assertion(spec, obs.as_ref()) {
            Ok(()) => return Ok(()),
            Err(message) => {
                if within_us == 0 || target.elapsed_us() >= deadline {
                    return Err(message);
                }
            }
        }
    }
}

/// Poll a CAN assertion until it holds or its `within` deadline passes.
async fn evaluate_expect<T: TestTarget + ?Sized>(
    spec: &ExpectStep,
    target: &mut T,
    test_deadline: Timestamp,
) -> Result<(), String> {
    let assertion = Assertion {
        kind: "frame",
        label: format!("0x{:03X}", spec.id),
        field: spec.signal.clone().unwrap_or_default(),
        present: spec.effective_present(),
        equals: spec.equals.clone(),
        greater_than: spec.greater_than,
        less_than: spec.less_than,
    };
    let id = spec.id;
    let signal_name = assertion.field.clone();
    poll_until(
        &assertion,
        spec.within.as_deref(),
        test_deadline,
        target,
        |t| {
            let signal_name = signal_name.clone();
            Box::pin(async move {
                let frame = t.poll(id).await.map_err(|e| e.to_string())?;
                let value =
                    match frame {
                        None => return Ok(None),
                        Some(_) if signal_name.is_empty() => None,
                        Some(f) => {
                            let message = t
                                .network()
                                .message(id)
                                .ok_or_else(|| format!("no message with id 0x{id:03X} in DBC"))?;
                            let signals = message
                                .decode_signals(&f.data)
                                .map_err(|e| format!("cannot decode 0x{id:03X}: {e}"))?;
                            let signal =
                                signals.iter().find(|s| s.name == signal_name).ok_or_else(
                                    || format!("unknown signal `{signal_name}` on 0x{id:03X}"),
                                )?;
                            Some(DecodedValue {
                                value: signal.value,
                                symbol: signal.symbol.clone(),
                            })
                        }
                    };
                Ok(Some(Observed { value }))
            })
        },
    )
    .await
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

/// Poll a netmap (UDP) assertion until it holds or its `within` deadline
/// passes. The message's destination endpoint (from the netmap) identifies the
/// traffic.
async fn evaluate_udp_expect<T: TestTarget + ?Sized>(
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
    let assertion = Assertion {
        kind: "message",
        label: format!("`{}`", spec.message),
        field: spec.field.clone().unwrap_or_default(),
        present: spec.effective_present(),
        equals: spec.equals.clone(),
        greater_than: spec.greater_than,
        less_than: spec.less_than,
    };
    let message_name = spec.message.clone();
    let field_name = assertion.field.clone();
    poll_until(
        &assertion,
        spec.within.as_deref(),
        test_deadline,
        target,
        |t| {
            let message = message.clone();
            let message_name = message_name.clone();
            let field_name = field_name.clone();
            Box::pin(async move {
                let dg = t.poll_msg(dst).await.map_err(|e| e.to_string())?;
                let value = match dg {
                    None => return Ok(None),
                    Some(_) if field_name.is_empty() => None,
                    Some(dg) => {
                        let decoded = message
                            .decode_field(&dg.payload, &field_name)
                            .map_err(|e| format!("cannot decode `{message_name}`: {e}"))?;
                        Some(DecodedValue {
                            value: decoded.value,
                            symbol: decoded.symbol,
                        })
                    }
                };
                Ok(Some(Observed { value }))
            })
        },
    )
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::target::{BoxFut, CanLink, NetmapLink, TargetError, POLL_US};
    use embrig_core::frame::CanFrame;
    use embrig_dbc::Network;
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

    fn decoded_obs(value: f64, symbol: Option<&str>) -> Observed {
        Observed {
            value: Some(DecodedValue {
                value,
                symbol: symbol.map(|s| s.to_string()),
            }),
        }
    }

    fn can_assertion(
        id: u32,
        signal: &str,
        present: Option<bool>,
        equals: Option<ExpectedValue>,
        greater_than: Option<f64>,
        less_than: Option<f64>,
    ) -> Assertion {
        Assertion {
            kind: "frame",
            label: format!("0x{id:03X}"),
            field: signal.to_string(),
            present,
            equals,
            greater_than,
            less_than,
        }
    }

    /// Decode `frame` into an [`Observed`] exactly like `evaluate_expect` does.
    fn can_observed(
        network: &Network,
        id: u32,
        signal: &str,
        frame: Option<&CanFrame>,
    ) -> Result<Option<Observed>, String> {
        let frame = match frame {
            Some(f) => f,
            None => return Ok(None),
        };
        let message = network
            .message(id)
            .ok_or_else(|| format!("no message with id 0x{id:03X} in DBC"))?;
        let signals = message
            .decode_signals(&frame.data)
            .map_err(|e| format!("cannot decode 0x{id:03X}: {e}"))?;
        let signal = signals
            .iter()
            .find(|s| s.name == signal)
            .ok_or_else(|| format!("unknown signal `{signal}` on 0x{id:03X}"))?;
        Ok(Some(Observed {
            value: Some(DecodedValue {
                value: signal.value,
                symbol: signal.symbol.clone(),
            }),
        }))
    }

    #[test]
    fn check_present() {
        let present = can_assertion(0x100, "", Some(true), None, None, None);
        assert!(check_assertion(&present, Some(&decoded_obs(400.0, None))).is_ok());
        let err = check_assertion(&present, None).unwrap_err();
        assert!(err.contains("present"), "got: {err}");

        let absent = can_assertion(0x100, "", Some(false), None, None, None);
        assert!(check_assertion(&absent, None).is_ok());
    }

    #[test]
    fn check_equals_numeric() {
        let pass = can_assertion(
            0x100,
            "voltage",
            None,
            Some(ExpectedValue::Num(400.0)),
            None,
            None,
        );
        assert!(check_assertion(&pass, Some(&decoded_obs(400.0, None))).is_ok());
        let fail = can_assertion(
            0x100,
            "voltage",
            None,
            Some(ExpectedValue::Num(401.0)),
            None,
            None,
        );
        let err = check_assertion(&fail, Some(&decoded_obs(400.0, None))).unwrap_err();
        assert!(err.contains("expected 401, got 400"), "got: {err}");
    }

    #[test]
    fn check_equals_bool() {
        let pass = can_assertion(
            0x220,
            "motor_enable",
            None,
            Some(ExpectedValue::Bool(true)),
            None,
            None,
        );
        assert!(check_assertion(&pass, Some(&decoded_obs(1.0, None))).is_ok());
        let fail = can_assertion(
            0x220,
            "motor_enable",
            None,
            Some(ExpectedValue::Bool(false)),
            None,
            None,
        );
        assert!(check_assertion(&fail, Some(&decoded_obs(1.0, None))).is_err());
    }

    #[test]
    fn check_equals_symbol() {
        let pass = can_assertion(
            0x100,
            "state",
            None,
            Some(ExpectedValue::Str("READY".into())),
            None,
            None,
        );
        assert!(check_assertion(&pass, Some(&decoded_obs(2.0, Some("READY")))).is_ok());
        let fail = can_assertion(
            0x100,
            "state",
            None,
            Some(ExpectedValue::Str("FAULT".into())),
            None,
            None,
        );
        assert!(check_assertion(&fail, Some(&decoded_obs(2.0, Some("READY")))).is_err());
    }

    #[test]
    fn check_comparisons() {
        let gt_pass = can_assertion(0x100, "voltage", None, None, Some(350.0), None);
        assert!(check_assertion(&gt_pass, Some(&decoded_obs(400.0, None))).is_ok());
        let gt_fail = can_assertion(0x100, "voltage", None, None, Some(450.0), None);
        let err = check_assertion(&gt_fail, Some(&decoded_obs(400.0, None))).unwrap_err();
        assert!(err.contains("expected > 450, got 400"), "got: {err}");
        let lt_pass = can_assertion(0x100, "voltage", None, None, None, Some(450.0));
        assert!(check_assertion(&lt_pass, Some(&decoded_obs(400.0, None))).is_ok());
    }

    #[test]
    fn check_unknown_signal_and_frame() {
        let n = network();
        let battery_frame = battery(400.0, "READY");
        let err = can_observed(&n, 0x100, "nope", Some(&battery_frame)).unwrap_err();
        assert!(err.contains("unknown signal"), "got: {err}");

        let missing = can_assertion(
            0x100,
            "voltage",
            None,
            Some(ExpectedValue::Num(400.0)),
            None,
            None,
        );
        let err = check_assertion(&missing, None).unwrap_err();
        assert!(err.contains("no frame"), "got: {err}");

        let err = can_observed(&n, 0x999, "voltage", Some(&frame(0x999, vec![0; 8]))).unwrap_err();
        assert!(err.contains("no message with id"), "got: {err}");
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

    impl CanLink for MockTarget {
        fn network(&self) -> &Network {
            &self.network
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

        fn send(&mut self, f: CanFrame) -> BoxFut<'_, Result<(), TargetError>> {
            Box::pin(async move {
                self.push_now(f);
                Ok(())
            })
        }

        fn poll(&mut self, id: u32) -> BoxFut<'_, Result<Option<CanFrame>, TargetError>> {
            Box::pin(async move {
                self.time += POLL_US;
                self.flush();
                Ok(self.visible.iter().rev().find(|f| f.id == id).cloned())
            })
        }
    }

    impl NetmapLink for MockTarget {}

    impl TestTarget for MockTarget {
        fn elapsed_us(&self) -> Timestamp {
            self.time
        }

        fn reset(&mut self) -> Result<(), TargetError> {
            self.time = 0;
            self.visible.clear();
            self.later.clear();
            Ok(())
        }

        fn wait(&mut self, duration: Timestamp) -> BoxFut<'_, Result<(), TargetError>> {
            Box::pin(async move {
                self.time += duration;
                self.flush();
                Ok(())
            })
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
    use embrig_net::MessageDef;

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

    fn udp_assertion(
        field: &str,
        present: Option<bool>,
        equals: Option<ExpectedValue>,
        greater_than: Option<f64>,
        less_than: Option<f64>,
    ) -> Assertion {
        Assertion {
            kind: "message",
            label: "`MotionState`".to_string(),
            field: field.to_string(),
            present,
            equals,
            greater_than,
            less_than,
        }
    }

    #[test]
    fn check_udp_equals_numeric_and_symbol() {
        let pass = udp_assertion("speed", None, Some(ExpectedValue::Num(1.5)), None, None);
        assert!(check_assertion(&pass, Some(&decoded_obs(1.5, None))).is_ok());
        let fail = udp_assertion("speed", None, Some(ExpectedValue::Num(2.0)), None, None);
        let err = check_assertion(&fail, Some(&decoded_obs(1.5, None))).unwrap_err();
        assert!(err.contains("expected 2, got 1.5"), "got: {err}");

        let symbol = udp_assertion(
            "state",
            None,
            Some(ExpectedValue::Str("DRIVING".into())),
            None,
            None,
        );
        assert!(check_assertion(&symbol, Some(&decoded_obs(1.0, Some("DRIVING")))).is_ok());
        let symbol_fail = udp_assertion(
            "state",
            None,
            Some(ExpectedValue::Str("STOPPED".into())),
            None,
            None,
        );
        assert!(check_assertion(&symbol_fail, Some(&decoded_obs(1.0, Some("DRIVING")))).is_err());
    }

    #[test]
    fn check_udp_comparisons_and_present() {
        let gt = udp_assertion("speed", None, None, Some(1.0), None);
        assert!(check_assertion(&gt, Some(&decoded_obs(1.5, None))).is_ok());
        let lt = udp_assertion("speed", None, None, None, Some(5.0));
        assert!(check_assertion(&lt, Some(&decoded_obs(1.5, None))).is_ok());
        let present = udp_assertion("", Some(true), None, None, None);
        assert!(check_assertion(&present, Some(&decoded_obs(1.5, None))).is_ok());
        assert!(check_assertion(&present, None).is_err());
    }

    #[tokio::test]
    async fn expect_udp_unknown_field_is_a_decode_error() {
        let mut target = MockUdpTarget::new();
        target.visible.push(udp_datagram(1.5, 0));
        let mut s = udp_spec("nope");
        s.equals = Some(ExpectedValue::Num(1.0));
        let err = evaluate_udp_expect(&s, &mut target, 1_000_000)
            .await
            .unwrap_err();
        assert!(err.contains("cannot decode"), "got: {err}");
    }

    /// A minimal UDP target: `poll_msg` advances time by a poll interval and
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

    impl CanLink for MockUdpTarget {
        fn network(&self) -> &Network {
            unimplemented!("no DBC network for a UDP-only mock")
        }
    }

    impl NetmapLink for MockUdpTarget {
        fn netmap(&self) -> Option<&Netmap> {
            Some(&self.netmap)
        }
        fn host(&self) -> Result<SocketAddr, TargetError> {
            Ok("127.0.0.1:5000".parse().unwrap())
        }
        fn send_msg(&mut self, dg: UdpDatagram) -> BoxFut<'_, Result<(), TargetError>> {
            Box::pin(async move {
                self.visible.push(dg);
                Ok(())
            })
        }
        fn poll_msg(
            &mut self,
            dst: SocketAddr,
        ) -> BoxFut<'_, Result<Option<UdpDatagram>, TargetError>> {
            Box::pin(async move {
                self.time += POLL_US;
                self.flush();
                Ok(self.visible.iter().rev().find(|d| d.dst == dst).cloned())
            })
        }
    }

    impl TestTarget for MockUdpTarget {
        fn elapsed_us(&self) -> Timestamp {
            self.time
        }
        fn reset(&mut self) -> Result<(), TargetError> {
            self.time = 0;
            self.visible.clear();
            self.later.clear();
            Ok(())
        }
        fn wait(&mut self, duration: Timestamp) -> BoxFut<'_, Result<(), TargetError>> {
            Box::pin(async move {
                self.time += duration;
                self.flush();
                Ok(())
            })
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
