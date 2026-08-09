//! The test runner: executes a [`TestSpec`] against a [`TestTarget`].
//!
//! Assertions are evaluated by polling the target. In virtual mode each poll
//! advances the simulation by a fixed poll interval, so `within` deadlines are
//! still deterministic; in hardware mode polls are wall-clock waits.

use openhil_core::fault::Fault;
use openhil_core::frame::CanFrame;
use openhil_core::signal::SignalValue;
use openhil_dbc::Network;
use thiserror::Error;

use crate::dsl::{parse_duration, ExpectStep, ExpectedValue, FaultKind, Step, TestSpec};
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
    Frame(#[from] openhil_core::frame::FrameError),
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
    let _budget = timeout;
    let start = target.elapsed_us();
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
            Step::Expect { spec } => evaluate_expect(spec, target).await,
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
        };

        match outcome {
            Ok(()) => {}
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

/// Poll an assertion until it holds or its `within` deadline passes.
async fn evaluate_expect<T: TestTarget>(spec: &ExpectStep, target: &mut T) -> Result<(), String> {
    let within = match &spec.within {
        Some(w) => parse_duration(w).map_err(|e| e.to_string())?,
        None => 0,
    };
    let deadline = target.elapsed_us().saturating_add(within);

    loop {
        let frame = target.poll(spec.id).await.map_err(|e| e.to_string())?;

        if spec.present == Some(false) && frame.is_some() {
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
    if let Some(present) = spec.present {
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
