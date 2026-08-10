//! Test execution targets.
//!
//! A [`TestTarget`] abstracts what a test runs against: either the
//! deterministic virtual simulation or a real SocketCAN bus. Hardware targets
//! exist only when the `socketcan` feature is enabled.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};

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
}

/// The interface the test runner drives.
///
/// Async trait methods keep the SocketCAN backend usable directly. The two
/// targets are used by value (no `dyn`), so auto-trait bounds are not a
/// concern here.
#[allow(async_fn_in_trait)]
pub trait TestTarget {
    /// The DBC network used to decode asserted signals.
    fn network(&self) -> &Network;
    /// Current time in microseconds (simulation time or bus time).
    fn elapsed_us(&self) -> Timestamp;
    /// Restore the target to a fresh state (faults, signal overrides and
    /// clock are discarded). Called before each test in a suite.
    fn reset(&mut self) -> Result<(), TargetError>;
    /// Override a signal of an ECU for its next transmission.
    fn set_signal(
        &mut self,
        ecu: &str,
        id: u32,
        signal: &str,
        value: SignalValue,
    ) -> Result<(), TargetError>;
    /// Inject a fault, optionally windowed.
    fn add_fault(
        &mut self,
        fault: Fault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError>;
    /// Transmit a frame on the bus.
    async fn send(&mut self, frame: CanFrame) -> Result<(), TargetError>;
    /// Advance time (virtual) or sleep (hardware).
    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError>;
    /// Advance a poll interval and return the most recent frame with `id`.
    async fn poll(&mut self, id: u32) -> Result<Option<CanFrame>, TargetError>;

    /// The netmap of a UDP network, if this target supports UDP.
    fn netmap(&self) -> Option<&Netmap> {
        None
    }

    /// The host endpoint used as the source of injected UDP datagrams.
    fn udp_host(&self) -> Result<SocketAddr, TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "UDP is not supported by this target".into(),
        ))
    }

    /// Transmit a UDP datagram. Targets without UDP support fail.
    async fn send_udp(&mut self, _dg: UdpDatagram) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "send_udp is not supported by this target".into(),
        ))
    }

    /// Poll for the most recent datagram delivered to `dst`.
    async fn poll_udp(&mut self, _dst: SocketAddr) -> Result<Option<UdpDatagram>, TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "poll_udp is not supported by this target".into(),
        ))
    }

    /// Override a field of a UDP message on an ECU.
    fn set_field(
        &mut self,
        _ecu: &str,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "set_field is not supported by this target".into(),
        ))
    }

    /// Inject a UDP fault, optionally windowed.
    fn add_fault_udp(
        &mut self,
        _fault: UdpFault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "add_fault_udp is not supported by this target".into(),
        ))
    }
}

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

impl TestTarget for VirtualTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.sim.time()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        *self = Self::new(&self.config, &self.dbc)?;
        Ok(())
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

    async fn send(&mut self, frame: CanFrame) -> Result<(), TargetError> {
        self.sim.inject(frame);
        Ok(())
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        self.sim.run_for(duration);
        Ok(())
    }

    async fn poll(&mut self, id: u32) -> Result<Option<CanFrame>, TargetError> {
        self.sim.run_for(POLL_US);
        Ok(self.sim.recorder().last_frame(id).cloned())
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
impl TestTarget for HardwareTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.bus.now_us()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        Ok(())
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

    async fn send(&mut self, frame: CanFrame) -> Result<(), TargetError> {
        self.bus
            .send(&frame)
            .await
            .map_err(|e| TargetError::Can(e.to_string()))?;
        Ok(())
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        tokio::time::sleep(std::time::Duration::from_micros(duration)).await;
        Ok(())
    }

    async fn poll(&mut self, id: u32) -> Result<Option<CanFrame>, TargetError> {
        let frame = self
            .bus
            .recv(std::time::Duration::from_micros(POLL_US))
            .await
            .map_err(|e| TargetError::Can(e.to_string()))?;
        Ok(frame.filter(|f| f.id == id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn tmp_file(name: &str, content: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("embrig-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn vehicle_yaml() -> std::path::PathBuf {
        tmp_file(
            "target-vehicle.yaml",
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

    fn vehicle_dbc() -> std::path::PathBuf {
        tmp_file(
            "target.dbc",
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
            serde_saphyr::from_str(&std::fs::read_to_string(vehicle_yaml()).unwrap()).unwrap();
        let mut target = VirtualTarget::new(&config, &vehicle_dbc()).unwrap();
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
