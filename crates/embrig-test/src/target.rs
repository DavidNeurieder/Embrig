//! Test execution targets.
//!
//! A [`TestTarget`] abstracts what a test runs against: either the
//! deterministic virtual simulation or a real SocketCAN bus. Hardware targets
//! exist only when the `socketcan` feature is enabled.

use std::collections::HashMap;
use std::future::Future;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::pin::Pin;

use embrig_core::ecu::EcuError;
use embrig_core::fault::{Fault, FaultRule};
use embrig_core::frame::CanFrame;
use embrig_core::signal::SignalValue;
use embrig_core::simulation::Simulation;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;
use embrig_models::{build_simulation_indexed, VehicleConfig};
use embrig_net::{Netmap, UdpDatagram, UdpFault};
use thiserror::Error;

/// Poll granularity: how far a `poll` advances virtual time (or how long a
/// hardware poll waits) before re-checking an assertion.
pub const POLL_US: Timestamp = 10_000;

/// A boxed, `Send` future borrowed from a target, making async trait methods
/// object-safe so targets can be held behind `dyn`.
pub type BoxFut<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

/// Errors produced while driving a test target.
#[derive(Debug, Error)]
pub enum TargetError {
    #[error("unknown ecu `{0}`")]
    UnknownEcu(String),
    #[error("ecu error: {0}")]
    Ecu(#[from] EcuError),
    #[error("invalid frame: {0}")]
    Frame(#[from] embrig_core::frame::FrameError),
    #[error("unsupported on hardware target: {0}")]
    UnsupportedOnHardware(String),
    #[error("unsupported on the system under test (SIL): {0}")]
    UnsupportedOnSut(String),
    #[error("unsupported on this target: {0}")]
    UnsupportedOnTarget(String),
    #[error("firmware under test failed a step: {0}")]
    SutTimeout(String),
    #[error("can error: {0}")]
    Can(String),
    #[error("net error: {0}")]
    Net(String),
    #[error("build failed: {0}")]
    Build(String),
}

/// The CAN link a target exposes: the DBC network plus the bus operations.
///
/// Methods default to a clear `UnsupportedOnTarget` error so a target that
/// only talks a message-map network (see [`NetmapLink`]) implements just
/// [`CanLink::network`].
pub trait CanLink {
    /// The DBC network used to decode asserted signals.
    fn network(&self) -> &Network;
    /// Override a signal of an ECU for its next transmission.
    fn set_signal(
        &mut self,
        _ecu: &str,
        _id: u32,
        _signal: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "set_signal is a CAN step; this target has no CAN bus".into(),
        ))
    }
    /// Inject a CAN fault, optionally windowed.
    fn add_fault(
        &mut self,
        _fault: Fault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "add_fault is a CAN step; this target has no CAN bus".into(),
        ))
    }
    /// Transmit a frame on the bus.
    fn send(&mut self, _frame: CanFrame) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async {
            Err(TargetError::UnsupportedOnTarget(
                "send is a CAN step; this target has no CAN bus".into(),
            ))
        })
    }
    /// Advance a poll interval and return the most recent frame with `id`.
    fn poll(&mut self, _id: u32) -> BoxFut<'_, Result<Option<CanFrame>, TargetError>> {
        Box::pin(async {
            Err(TargetError::UnsupportedOnTarget(
                "poll is a CAN step; this target has no CAN bus".into(),
            ))
        })
    }
}

/// A message-map network link (UDP today, TCP later).
///
/// The names are transport neutral: `host` is the source of injected messages,
/// `send_msg`/`poll_msg` move payloads keyed by destination endpoint, and
/// `add_fault` injects link faults. Targets without a netmap network inherit
/// clear error defaults.
pub trait NetmapLink {
    /// The netmap of the message-map network, if this target has one.
    fn netmap(&self) -> Option<&Netmap> {
        None
    }

    /// The host endpoint used as the source of injected messages.
    fn host(&self) -> Result<SocketAddr, TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "netmap messaging is not supported by this target".into(),
        ))
    }

    /// Transmit a message to `dst`. Targets without a message-map network fail.
    fn send_msg(&mut self, _dg: UdpDatagram) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async {
            Err(TargetError::UnsupportedOnTarget(
                "netmap messaging is not supported by this target".into(),
            ))
        })
    }

    /// Poll for the most recent message delivered to `dst`.
    fn poll_msg(
        &mut self,
        _dst: SocketAddr,
    ) -> BoxFut<'_, Result<Option<UdpDatagram>, TargetError>> {
        Box::pin(async {
            Err(TargetError::UnsupportedOnTarget(
                "netmap messaging is not supported by this target".into(),
            ))
        })
    }

    /// Override a field of a message on an ECU.
    fn set_field(
        &mut self,
        _ecu: &str,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "netmap messaging is not supported by this target".into(),
        ))
    }

    /// Inject a link fault, optionally windowed.
    fn add_fault(
        &mut self,
        _fault: UdpFault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "netmap messaging is not supported by this target".into(),
        ))
    }
}

/// The interface the test runner drives: a CAN link plus a message-map
/// network link, together with the lifecycle methods common to every target.
///
/// Async methods return boxed futures, so [`TestTarget`] is object safe and
/// can be held behind `dyn` (see [`DynTestTarget`]).
pub trait TestTarget: CanLink + NetmapLink {
    /// Current time in microseconds (simulation time or bus time).
    fn elapsed_us(&self) -> Timestamp;
    /// Restore the target to a fresh state (faults, signal overrides and
    /// clock are discarded). Called before each test in a suite.
    fn reset(&mut self) -> Result<(), TargetError>;
    /// Advance time (virtual) or sleep (hardware).
    fn wait(&mut self, duration: Timestamp) -> BoxFut<'_, Result<(), TargetError>>;

    /// Inject a CAN fault, forwarding to [`CanLink::add_fault`] to keep the
    /// two same-named link methods distinct at call sites.
    fn add_can_fault(
        &mut self,
        fault: Fault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        CanLink::add_fault(self, fault, start, duration)
    }

    /// Inject a link fault, forwarding to [`NetmapLink::add_fault`].
    fn add_netmap_fault(
        &mut self,
        fault: UdpFault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        NetmapLink::add_fault(self, fault, start, duration)
    }
}

/// Object-safe form of [`TestTarget`], used when targets are held behind
/// `dyn` (e.g. the protocol registry).
pub trait DynTestTarget: TestTarget {}

impl<T: TestTarget> DynTestTarget for T {}

/// A test target running against the deterministic virtual simulation.
pub struct VirtualTarget {
    sim: Simulation,
    ecus: HashMap<String, usize>,
    network: Network,
    config: VehicleConfig,
    dbc: PathBuf,
}

impl VirtualTarget {
    /// Build a virtual target from a vehicle config and DBC file.
    pub fn new(config: &VehicleConfig, dbc_path: &Path) -> Result<Self, TargetError> {
        let text = std::fs::read_to_string(dbc_path).map_err(|e| {
            TargetError::Can(format!("failed to read DBC `{}`: {e}", dbc_path.display()))
        })?;
        let network =
            embrig_dbc::parse(&text).map_err(|e| TargetError::Can(format!("invalid DBC: {e}")))?;
        let built = build_simulation_indexed(config, dbc_path)
            .map_err(|e| TargetError::Can(e.to_string()))?;
        Ok(Self {
            sim: built.sim,
            ecus: built.ecus.into_iter().collect(),
            network,
            config: config.clone(),
            dbc: dbc_path.to_path_buf(),
        })
    }

    /// Access the underlying simulation (for reports).
    pub fn sim(&self) -> &Simulation {
        &self.sim
    }
}

impl CanLink for VirtualTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn set_signal(
        &mut self,
        ecu: &str,
        id: u32,
        signal: &str,
        value: SignalValue,
    ) -> Result<(), TargetError> {
        let index = self
            .ecus
            .get(ecu)
            .ok_or_else(|| TargetError::UnknownEcu(ecu.to_string()))?;
        self.sim.set_signal(*index, id, signal, value)?;
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

    fn send(&mut self, frame: CanFrame) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async move {
            self.sim.inject(frame);
            Ok(())
        })
    }

    fn poll(&mut self, id: u32) -> BoxFut<'_, Result<Option<CanFrame>, TargetError>> {
        Box::pin(async move {
            self.sim.run_for(POLL_US);
            Ok(self.sim.recorder().last_message(&id).cloned())
        })
    }
}

impl NetmapLink for VirtualTarget {}

impl TestTarget for VirtualTarget {
    fn elapsed_us(&self) -> Timestamp {
        self.sim.time()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        *self = Self::new(&self.config, &self.dbc)?;
        Ok(())
    }

    fn wait(&mut self, duration: Timestamp) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async move {
            self.sim.run_for(duration);
            Ok(())
        })
    }
}

/// A test target running against a real SocketCAN interface.
///
/// `set_signal` and faults are unsupported: there is no software router in the
/// loop, so the runner fails with a clear error instead of silently ignoring
/// the step.
#[cfg(feature = "socketcan")]
pub struct HardwareTarget {
    bus: embrig_can::SocketCanBus,
    network: Network,
}

#[cfg(feature = "socketcan")]
impl HardwareTarget {
    pub fn new(interface: &str, network: Network) -> Result<Self, TargetError> {
        let bus = embrig_can::SocketCanBus::open(interface)
            .map_err(|e| TargetError::Can(e.to_string()))?;
        Ok(Self { bus, network })
    }
}

#[cfg(feature = "socketcan")]
impl CanLink for HardwareTarget {
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
        Err(TargetError::UnsupportedOnHardware(
            "set_signal cannot inject into a live bus".into(),
        ))
    }

    fn add_fault(
        &mut self,
        _fault: Fault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "faults require the virtual router".into(),
        ))
    }

    fn send(&mut self, frame: CanFrame) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async move {
            self.bus
                .send(&frame)
                .await
                .map_err(|e| TargetError::Can(e.to_string()))?;
            Ok(())
        })
    }

    fn poll(&mut self, id: u32) -> BoxFut<'_, Result<Option<CanFrame>, TargetError>> {
        Box::pin(async move {
            let frame = self
                .bus
                .recv(std::time::Duration::from_micros(POLL_US))
                .await
                .map_err(|e| TargetError::Can(e.to_string()))?;
            Ok(frame.filter(|f| f.id == id))
        })
    }
}

#[cfg(feature = "socketcan")]
impl NetmapLink for HardwareTarget {}

#[cfg(feature = "socketcan")]
impl TestTarget for HardwareTarget {
    fn elapsed_us(&self) -> Timestamp {
        self.bus.now_us()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        Ok(())
    }

    fn wait(&mut self, duration: Timestamp) -> BoxFut<'_, Result<(), TargetError>> {
        Box::pin(async move {
            tokio::time::sleep(std::time::Duration::from_micros(duration)).await;
            Ok(())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[tokio::test]
    async fn dyn_target_is_object_safe_and_driveable() {
        let config: VehicleConfig =
            serde_saphyr::from_str(&std::fs::read_to_string(vehicle_yaml("dyn")).unwrap()).unwrap();
        let mut target: Box<dyn DynTestTarget> =
            Box::new(VirtualTarget::new(&config, &vehicle_dbc("dyn")).unwrap());
        target.wait(50_000).await.unwrap();
        assert_eq!(target.elapsed_us(), 50_000);
        target
            .set_signal("battery", 0x100, "voltage", SignalValue::Num(460.0))
            .unwrap();
        target.wait(100_000).await.unwrap();
        let frame = target.poll(0x100).await.unwrap().unwrap();
        let value = target
            .network()
            .message(0x100)
            .unwrap()
            .decode_signal(&frame.data, "voltage")
            .unwrap();
        assert!((value - 460.0).abs() < 1e-6);
        target
            .add_can_fault(Fault::DropFrame { id: 0x100 }, Some(0), Some(10_000))
            .unwrap();
        assert!(target.netmap().is_none());
        assert!(target.host().is_err());
    }

    fn tmp_file(name: &str, content: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("embrig-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn vehicle_yaml(tag: &str) -> std::path::PathBuf {
        tmp_file(
            &format!("target-{tag}.yaml"),
            r#"
name: demo
dbc: target.dbc
step_us: 1000
ecus:
  - name: battery
    type: config
    message: BatteryStatus
    period_us: 100000
    signals:
      voltage: 400.0
      state: "READY"
interfaces:
  - name: virtual
    type: virtual
"#,
        )
    }

    fn vehicle_dbc(tag: &str) -> std::path::PathBuf {
        tmp_file(
            &format!("target-{tag}.dbc"),
            r#"VERSION ""

NS_ :

BS_:

BU_: engine

BO_ 256 BatteryStatus: 8 engine
 SG_ voltage : 0|16@1+ (0.1,0) [0|600] "V" engine
 SG_ state : 16|4@1+ (1,0) [0|4] "" engine

BO_ 544 MotorEnable: 8 engine
 SG_ motor_enable : 0|1@1+ (1,0) [0|1] "" engine

VAL_ 256 state 0 "OFF" 1 "INIT" 2 "READY" 3 "CHARGING" 4 "FAULT" ;
"#,
        )
    }

    #[tokio::test]
    async fn virtual_target_drives_signals_and_polls() {
        let config: VehicleConfig =
            serde_saphyr::from_str(&std::fs::read_to_string(vehicle_yaml("drive")).unwrap())
                .unwrap();
        let mut target = VirtualTarget::new(&config, &vehicle_dbc("drive")).unwrap();
        target.wait(50_000).await.unwrap();
        assert_eq!(target.elapsed_us(), 50_000);
        target
            .set_signal("battery", 0x100, "voltage", SignalValue::Num(460.0))
            .unwrap();
        target.wait(100_000).await.unwrap();
        let frame = target.poll(0x100).await.unwrap().expect("battery frame");
        let network = target.network();
        assert!(
            (network
                .message(0x100)
                .unwrap()
                .decode_signal(&frame.data, "voltage")
                .unwrap()
                - 460.0)
                .abs()
                < 1e-6
        );
    }

    #[test]
    fn virtual_target_rejects_unknown_ecu() {
        let config: VehicleConfig = serde_saphyr::from_str(
            &std::fs::read_to_string(tmp_file(
                "target2-vehicle.yaml",
                r#"
name: demo
dbc: target2.dbc
step_us: 1000
ecus:
  - name: battery
    type: config
    message: BatteryStatus
    period_us: 100000
    signals:
      voltage: 400.0
      state: "READY"
interfaces:
  - name: virtual
    type: virtual
"#,
            ))
            .unwrap(),
        )
        .unwrap();
        let mut target = VirtualTarget::new(
            &config,
            &tmp_file(
                "target2.dbc",
                r#"VERSION ""

NS_ :

BS_:

BU_: engine

BO_ 256 BatteryStatus: 8 engine
 SG_ voltage : 0|16@1+ (0.1,0) [0|600] "V" engine
 SG_ state : 16|4@1+ (1,0) [0|4] "" engine

VAL_ 256 state 0 "OFF" 1 "INIT" 2 "READY" 3 "CHARGING" 4 "FAULT" ;
"#,
            ),
        )
        .unwrap();
        assert!(matches!(
            target.set_signal("nope", 0x100, "voltage", SignalValue::Num(1.0)),
            Err(TargetError::UnknownEcu(_))
        ));
    }
}
