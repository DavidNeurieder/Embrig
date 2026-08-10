//! Software-in-the-loop: run host-compiled firmware against the virtual bus.
//!
//! A [`SilRegistry`] maps the `type: sil` ECU names in `vehicle.yaml` to the
//! firmware that implements them. Firmware is ordinary Rust implementing the
//! [`Ecu`] trait — the same interface the built-in vECUs use — but compiled
//! for the host, so a suite runs against the real firmware without hardware.
//!
//! [`SilTarget`] is a [`TestTarget`] (drop-in for `embrig_test::VirtualTarget`)
//! so the exact same YAML suites run against virtual ECUs, SIL firmware and —
//! later — hardware. [`sil_run`] is the one-call helper:
//!
//! ```no_run
//! # use embrig_sil::{sil_run, SilRegistry};
//! # use embrig_core::ecu::{Ecu, EcuError};
//! # use embrig_core::time::Timestamp;
//! # use embrig_core::frame::CanFrame;
//! # struct NoopFirmware;
//! # impl Ecu for NoopFirmware {
//! #     fn name(&self) -> &str { "noop" }
//! # }
//! # let config: embrig_models::VehicleConfig = unimplemented!();
//! # let dbc: &std::path::Path = unimplemented!();
//! let mut registry = SilRegistry::new();
//! registry.register(
//!     "controller",
//!     |_name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
//!         Ok(Box::new(NoopFirmware))
//!     },
//! );
//! # let suites: Vec<std::path::PathBuf> = vec![];
//! let result = sil_run(&config, &dbc, registry, &suites).unwrap();
//! ```
//!
//! Firmware runs under a wall-clock step budget (default 100 ms per simulated
//! step, override with `step_budget_us` on the ECU). An overrun fails the
//! test with [`TargetError::SutTimeout`]; because each test gets a fresh
//! simulation, a hung step never leaks into the next test. Signals of the
//! firmware itself cannot be overridden ([`TargetError::UnsupportedOnSut`]) —
//! drive it through the bus — while faults and config-node signal overrides
//! behave exactly as in virtual mode.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::Instant;

use embrig_core::ecu::{Ecu, EcuError};
use embrig_core::fault::{Fault, FaultRule};
use embrig_core::frame::CanFrame;
use embrig_core::signal::SignalValue;
use embrig_core::simulation::Simulation;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;
use embrig_models::{build_simulation_indexed_with, EcuFactory, ModelError, VehicleConfig};
use embrig_test::target::POLL_US;
use embrig_test::{run_suite, SuiteResult, TargetError, TestError, TestTarget};
use thiserror::Error;

/// Errors from the SIL toolchain.
#[derive(Debug, Error)]
pub enum SilError {
    #[error("{0}")]
    Target(#[from] TargetError),
    #[error("{0}")]
    Test(#[from] TestError),
    #[error("{0}")]
    Model(#[from] ModelError),
    #[error("{0}")]
    Io(#[from] std::io::Error),
}

/// A firmware factory registry keyed by the `type: sil` ECU name.
///
/// Factories are looked up (and re-invoked) every time the simulation is
/// built, i.e. once at startup and again on each test reset — so firmware
/// state never leaks between tests.
#[derive(Default)]
pub struct SilRegistry {
    factories: HashMap<String, Box<dyn EcuFactory>>,
}

impl SilRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register the firmware for an ECU. `factory` may be any [`EcuFactory`],
    /// including a plain closure.
    pub fn register(
        &mut self,
        name: impl Into<String>,
        factory: impl EcuFactory + 'static,
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

impl EcuFactory for SilRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn Ecu>, EcuError> {
        let factory = self.factories.get(name).ok_or_else(|| {
            EcuError::NotRegistered(format!(
                "`{name}` (registered: {})",
                self.names().join(", ")
            ))
        })?;
        let inner = factory.create(name, step_budget_us)?;
        Ok(Box::new(BudgetedEcu::new(inner, step_budget_us)))
    }
}

impl EcuFactory for &SilRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn Ecu>, EcuError> {
        SilRegistry::create(self, name, step_budget_us)
    }
}

/// Wall-clock budget enforcement around one firmware ECU.
///
/// Panics if a single `update`/`on_message` call takes longer than
/// `budget_us`; [`SilTarget`] converts that panic into a test failure. The
/// simulation is rebuilt per test, so a panicked step discards the firmware
/// state and the next test starts clean.
struct BudgetedEcu {
    inner: Box<dyn Ecu>,
    budget_us: u64,
    step_start: Instant,
}

impl BudgetedEcu {
    fn new(inner: Box<dyn Ecu>, budget_us: u64) -> Self {
        Self {
            inner,
            budget_us,
            step_start: Instant::now(),
        }
    }
}

impl Ecu for BudgetedEcu {
    fn name(&self) -> &str {
        self.inner.name()
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        self.step_start = Instant::now();
        self.inner.update(time, out);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn on_message(&mut self, frame: &CanFrame, time: Timestamp) {
        self.step_start = Instant::now();
        self.inner.on_message(frame, time);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn set_signal(&mut self, id: u32, signal: &str, value: SignalValue) -> Result<(), EcuError> {
        self.inner.set_signal(id, signal, value)
    }
}

fn check_budget(name: &str, budget_us: u64, start: Instant) {
    let took_us = start.elapsed().as_micros() as u64;
    if took_us > budget_us {
        panic!("firmware `{name}` exceeded its {budget_us}µs step budget (took {took_us}µs)");
    }
}

/// A [`TestTarget`] running the YAML suites against host-compiled firmware.
///
/// Mirrors [`embrig_test::VirtualTarget`], except: signal overrides on the
/// firmware itself are rejected (drive it via the bus), and a firmware step
/// exceeding its budget fails the test.
pub struct SilTarget {
    sim: Simulation,
    ecus: HashMap<String, usize>,
    sut: HashSet<String>,
    network: Network,
    config: VehicleConfig,
    dbc: PathBuf,
    registry: SilRegistry,
}

impl SilTarget {
    /// Build a SIL target. `registry` is owned so `reset` can re-invoke the
    /// firmware factories for a fresh simulation.
    pub fn new(
        config: &VehicleConfig,
        dbc_path: &Path,
        registry: SilRegistry,
    ) -> Result<Self, TargetError> {
        let text = std::fs::read_to_string(dbc_path).map_err(|e| {
            TargetError::Can(format!("failed to read DBC `{}`: {e}", dbc_path.display()))
        })?;
        let network =
            embrig_dbc::parse(&text).map_err(|e| TargetError::Can(format!("invalid DBC: {e}")))?;
        let built = build_simulation_indexed_with(config, dbc_path, &registry)
            .map_err(|e| TargetError::Can(e.to_string()))?;
        let sut = config
            .ecus
            .iter()
            .filter(|e| e.kind == embrig_models::EcuKind::Sil)
            .map(|e| e.name.clone())
            .collect();
        Ok(Self {
            sim: built.sim,
            ecus: built.ecus.into_iter().collect(),
            sut,
            network,
            config: config.clone(),
            dbc: dbc_path.to_path_buf(),
            registry,
        })
    }

    /// Access the underlying simulation (for reports).
    pub fn sim(&self) -> &Simulation {
        &self.sim
    }

    /// The firmware registry (for diagnostics).
    pub fn registry(&self) -> &SilRegistry {
        &self.registry
    }

    /// Run a firmware step, converting a firmware panic (e.g. a budget
    /// overrun) into a graceful test failure.
    fn run_sim<F, T>(&mut self, f: F) -> Result<T, TargetError>
    where
        F: FnOnce(&mut Simulation) -> T,
    {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&mut self.sim))).map_err(
            |payload| {
                let message = payload
                    .downcast_ref::<&str>()
                    .copied()
                    .or_else(|| payload.downcast_ref::<String>().map(|s| s.as_str()))
                    .unwrap_or("firmware under test panicked during a simulated step");
                TargetError::SutTimeout(message.to_string())
            },
        )
    }
}

impl TestTarget for SilTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.sim.time()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        let built = build_simulation_indexed_with(&self.config, &self.dbc, &self.registry)
            .map_err(|e| TargetError::Can(e.to_string()))?;
        self.sim = built.sim;
        self.ecus = built.ecus.into_iter().collect();
        Ok(())
    }

    fn set_signal(
        &mut self,
        ecu: &str,
        id: u32,
        signal: &str,
        value: SignalValue,
    ) -> Result<(), TargetError> {
        if self.sut.contains(ecu) {
            return Err(TargetError::UnsupportedOnSut(format!(
                "cannot override signals on firmware `{ecu}`; drive it via the bus instead"
            )));
        }
        let index = *self
            .ecus
            .get(ecu)
            .ok_or_else(|| TargetError::UnknownEcu(ecu.to_string()))?;
        self.run_sim(|sim| sim.set_signal(index, id, signal, value))??;
        Ok(())
    }

    fn add_fault(
        &mut self,
        fault: Fault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        self.sim.add_fault(FaultRule {
            fault,
            start: start.unwrap_or(self.sim.time()),
            duration,
        });
        Ok(())
    }

    async fn send(&mut self, frame: CanFrame) -> Result<(), TargetError> {
        self.run_sim(|sim| sim.inject(frame))?;
        Ok(())
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        self.run_sim(|sim| sim.run_for(duration))?;
        Ok(())
    }

    async fn poll(&mut self, id: u32) -> Result<Option<CanFrame>, TargetError> {
        self.run_sim(|sim| sim.run_for(POLL_US))?;
        Ok(self.sim.recorder().last_frame(id).cloned())
    }
}

/// Run the YAML test files against the firmware in `registry`.
///
/// Convenience wrapper over [`SilTarget`] + [`run_suite`]; use the building
/// blocks directly if you need to reuse a target or registry.
pub fn sil_run(
    config: &VehicleConfig,
    dbc_path: &Path,
    registry: SilRegistry,
    files: &[PathBuf],
) -> Result<SuiteResult, SilError> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut target = SilTarget::new(config, dbc_path, registry)?;
        run_suite(&mut target, files, "sil")
            .await
            .map_err(SilError::from)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    static NEXT_TMP: AtomicUsize = AtomicUsize::new(0);

    const DBC: &str = r#"VERSION "0.1"

NS_ :

BS_:

BU_: Vector__XXX

BO_ 256 SensorInput: 8 Vector__XXX
 SG_ temperature : 0|16@1+ (0.1,0) [0|200] ""  Vector__XXX

BO_ 512 ValveCommand: 8 Vector__XXX
 SG_ valve_open : 0|1@1+ (1,0) [0|1] ""  Vector__XXX

VAL_ 512 valve_open 0 "CLOSED" 1 "OPEN" ;
"#;

    fn tmp_file(name: &str, content: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "embrig-sil-{}-{}",
            std::process::id(),
            NEXT_TMP.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn vehicle_yaml() -> std::path::PathBuf {
        tmp_file(
            "sil-vehicle.yaml",
            r#"
name: thermal
dbc: sil-test.dbc
step_us: 1000
ecus:
  - name: sensor
    type: config
    message: SensorInput
    period_us: 100000
    signals:
      temperature: 45.0
  - name: controller
    type: sil
    period_us: 50000
    listen: [0x100]
    step_budget_us: 100000
interfaces:
  - name: virtual
    type: virtual
  - name: sil
    type: sil
"#,
        )
    }

    fn vehicle_dbc() -> std::path::PathBuf {
        tmp_file("sil-test.dbc", DBC)
    }

    fn config() -> VehicleConfig {
        serde_saphyr::from_str(&std::fs::read_to_string(vehicle_yaml()).unwrap()).unwrap()
    }

    fn suite(name: &str, steps: &str) -> std::path::PathBuf {
        tmp_file(name, &format!("name: {name}\ntimeout: 5s\nsteps:\n{steps}"))
    }

    /// Decides `valve_open` from the latest temperature on 0x100.
    struct TestFirmware {
        name: String,
        temperature: f64,
        next_tx: Timestamp,
        open: bool,
    }

    impl TestFirmware {
        fn new(name: &str) -> Self {
            Self {
                name: name.to_string(),
                temperature: 0.0,
                next_tx: 0,
                open: false,
            }
        }
    }

    impl Ecu for TestFirmware {
        fn name(&self) -> &str {
            &self.name
        }

        fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
            if frame.id == 0x100 {
                let network = embrig_dbc::parse(DBC).unwrap();
                self.temperature = network
                    .message(0x100)
                    .unwrap()
                    .decode_signal(&frame.data, "temperature")
                    .unwrap_or(self.temperature);
                self.open = (10.0..=90.0).contains(&self.temperature);
            }
        }

        fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
            if time >= self.next_tx {
                let network = embrig_dbc::parse(DBC).unwrap();
                let data = network
                    .message(0x200)
                    .unwrap()
                    .encode_signals(&[("valve_open", if self.open { 1.0 } else { 0.0 })])
                    .unwrap();
                out.push(CanFrame::new(0x200, data).unwrap());
                self.next_tx = time + 50_000;
            }
        }
    }

    fn registry() -> SilRegistry {
        let mut registry = SilRegistry::new();
        registry.register(
            "controller",
            |name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
                Ok(Box::new(TestFirmware::new(name)))
            },
        );
        registry
    }

    #[test]
    fn nominal_suite_passes_and_overrange_closes_valve() {
        let config = config();
        let dbc = vehicle_dbc();
        let files = vec![
            suite(
                "valid_temperature_opens_valve.yaml",
                "  - wait: { time: 300ms }\n  - expect: { id: 0x200, signal: valve_open, equals: true, within: 1s }\n",
            ),
            suite(
                "overrange_temperature_closes_valve.yaml",
                "  - wait: { time: 300ms }\n  - set_signal: { ecu: sensor, id: 0x100, signal: temperature, value: 150.0 }\n  - expect: { id: 0x200, signal: valve_open, equals: false, within: 1s }\n",
            ),
        ];
        let result = sil_run(&config, &dbc, registry(), &files).unwrap();
        let failures: Vec<String> = result
            .tests
            .iter()
            .flat_map(|t| t.failures.iter().cloned())
            .collect();
        assert_eq!(result.failed(), 0, "failures: {failures:?}");
    }

    #[tokio::test]
    async fn budget_overrun_fails_the_step() {
        let config = config();
        let dbc = vehicle_dbc();
        let mut registry = SilRegistry::new();
        registry.register(
            "controller",
            |name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
                struct Slow {
                    name: String,
                }
                impl Ecu for Slow {
                    fn name(&self) -> &str {
                        &self.name
                    }
                    fn update(&mut self, _time: Timestamp, _out: &mut Vec<CanFrame>) {
                        std::thread::sleep(std::time::Duration::from_millis(200));
                    }
                }
                Ok(Box::new(Slow {
                    name: name.to_string(),
                }))
            },
        );
        let mut target = SilTarget::new(&config, &dbc, registry).unwrap();
        let err = target.wait(50_000).await.unwrap_err();
        assert!(matches!(err, TargetError::SutTimeout(_)), "got: {err:?}");
        // The next test starts from a fresh simulation: reset succeeds.
        target.reset().unwrap();
    }

    #[tokio::test]
    async fn sut_signal_override_is_rejected_but_config_signals_work() {
        let mut target = SilTarget::new(&config(), &vehicle_dbc(), registry()).unwrap();
        assert!(matches!(
            target.set_signal("controller", 0x100, "temperature", SignalValue::Num(30.0)),
            Err(TargetError::UnsupportedOnSut(_))
        ));
        target
            .set_signal("sensor", 0x100, "temperature", SignalValue::Num(30.0))
            .unwrap();
        target.wait(100_000).await.unwrap();
        let frame = target.poll(0x100).await.unwrap().expect("sensor frame");
        let value = target
            .network()
            .message(0x100)
            .unwrap()
            .decode_signal(&frame.data, "temperature")
            .unwrap();
        assert!((value - 30.0).abs() < 1e-6);
    }

    #[tokio::test]
    async fn faults_can_be_injected_on_the_sil_bus() {
        let mut target = SilTarget::new(&config(), &vehicle_dbc(), registry()).unwrap();
        target
            .add_fault(Fault::DropFrame { id: 0x200 }, Some(0), Some(500_000))
            .unwrap();
        target.wait(300_000).await.unwrap();
        assert!(
            target.poll(0x200).await.unwrap().is_none(),
            "valve frames must be dropped by the fault"
        );
    }

    #[test]
    fn firmware_factories_are_rerun_on_reset() {
        let instances = Arc::new(AtomicUsize::new(0));
        let mut registry = SilRegistry::new();
        let counter = instances.clone();
        registry.register(
            "controller",
            move |name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
                counter.fetch_add(1, Ordering::SeqCst);
                Ok(Box::new(TestFirmware::new(name)))
            },
        );
        let mut target = SilTarget::new(&config(), &vehicle_dbc(), registry).unwrap();
        assert_eq!(instances.load(Ordering::SeqCst), 1);
        target.reset().unwrap();
        target.reset().unwrap();
        assert_eq!(instances.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn unknown_sil_ecu_fails_with_a_clear_error() {
        let err = match SilTarget::new(&config(), &vehicle_dbc(), SilRegistry::new()) {
            Ok(_) => panic!("expected a clear startup error"),
            Err(e) => e,
        };
        let message = err.to_string();
        assert!(
            message.contains("no firmware registered for SIL ECU `controller`"),
            "got: {message}"
        );
        assert!(message.contains("registered"), "got: {message}");
    }
}
