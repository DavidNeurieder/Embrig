//! UDP test targets.
//!
//! These targets drive the same YAML suites as the CAN targets, but over
//! Ethernet (UDP/IP). A [`UdpTarget`] runs against the deterministic virtual
//! network ([`UdpSim`]), [`UdpSutTarget`] runs host-compiled firmware in the
//! loop (`udp-sil` ECUs), and [`UdpHardwareTarget`] talks to a real Ethernet
//! link through a bound UDP socket. All three resolve netmap message names to
//! endpoints exactly like the virtual simulation does, so a suite runs
//! unchanged across virtual, SIL and hardware.
//!
//! Messages are identified by their destination endpoint. `expect_udp` and
//! `fault_udp` steps key by netmap message name; the host endpoint (from the
//! network config) is the source of injected datagrams and the destination of
//! telemetry.

use std::collections::{HashMap, HashSet, VecDeque};
use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;
use embrig_models::{EthEcuKind, NetworkConfig, SignalLiteral, VehicleConfig};
use embrig_net::{Netmap, UdpDatagram, UdpEcuFactory, UdpFault, UdpFaultRule, UdpRegistry, UdpSim};
use thiserror::Error;

use crate::target::{TargetError, TestTarget, POLL_US};
use crate::{run_suite, SuiteResult, TestError};

/// Errors from the UDP toolchain.
#[derive(Debug, Error)]
pub enum UdpError {
    #[error("{0}")]
    Target(#[from] TargetError),
    #[error("{0}")]
    Test(#[from] TestError),
    #[error("{0}")]
    Io(#[from] std::io::Error),
}

/// A built UDP simulation plus the ECU name → index map needed to override
/// fields by name at runtime.
pub struct BuiltUdpSim {
    pub sim: UdpSim,
    pub ecus: Vec<(String, usize)>,
}

/// Build a UDP simulation from a vehicle config, network config and netmap.
///
/// Config ECUs (`type: udp-config`) transmit their netmap message on a fixed
/// period; `type: udp-sil` ECUs are created through `factories` (see
/// [`UdpRegistry`]). ECUs are attached in config order, which keeps the
/// simulation deterministic.
pub fn build_udp_simulation_with(
    config: &VehicleConfig,
    net_config: &NetworkConfig,
    netmap: &Netmap,
    factories: impl UdpEcuFactory,
) -> Result<BuiltUdpSim, TargetError> {
    let _ = net_config;
    let mut sim = UdpSim::new(config.step_us);
    let mut ecus = Vec::with_capacity(config.eth_ecus.len());

    for cfg in &config.eth_ecus {
        let index = match cfg.kind {
            EthEcuKind::UdpConfig => {
                let msg_name = cfg.message.as_ref().ok_or_else(|| {
                    TargetError::Net(format!("eth ECU `{}` has no `message` name", cfg.name))
                })?;
                let message = netmap.message(msg_name).cloned().ok_or_else(|| {
                    TargetError::Net(format!("message `{msg_name}` not found in netmap"))
                })?;
                let base = literal_fields(&message, &cfg.fields)?;
                let ecu = embrig_net::UdpConfigEcu::new(
                    cfg.name.clone(),
                    cfg.address,
                    msg_name,
                    message,
                    cfg.period_us,
                    base,
                );
                sim.attach(Box::new(ecu), cfg.address)
            }
            EthEcuKind::UdpSil => {
                let ecu = factories
                    .create(&cfg.name, cfg.step_budget_us)
                    .map_err(|e| TargetError::Net(format!("eth ECU `{}`: {e}", cfg.name)))?;
                sim.attach(ecu, cfg.address)
            }
        };
        ecus.push((cfg.name.clone(), index));
    }

    Ok(BuiltUdpSim { sim, ecus })
}

/// Like [`build_udp_simulation_with`] but no firmware is registered, so any
/// `udp-sil` ECU fails to build.
pub fn build_udp_simulation(
    config: &VehicleConfig,
    net_config: &NetworkConfig,
    netmap: &Netmap,
) -> Result<BuiltUdpSim, TargetError> {
    build_udp_simulation_with(config, net_config, netmap, UdpRegistry::new())
}

/// Convert YAML field literals (numeric, boolean or symbolic) to signal
/// values, resolving symbols through the netmap message.
fn literal_fields(
    message: &embrig_net::MessageDef,
    fields: &std::collections::BTreeMap<String, SignalLiteral>,
) -> Result<std::collections::BTreeMap<String, SignalValue>, TargetError> {
    let mut base = std::collections::BTreeMap::new();
    for (field, lit) in fields {
        let value = match lit {
            SignalLiteral::Num(v) => SignalValue::Num(*v),
            SignalLiteral::Bool(b) => SignalValue::Num(if *b { 1.0 } else { 0.0 }),
            SignalLiteral::Str(s) => {
                let f = message.field(field).ok_or_else(|| {
                    TargetError::Net(format!(
                        "unknown field `{field}` on `{}`",
                        message_name(message)
                    ))
                })?;
                let raw = message.resolve_symbol(field, s).ok_or_else(|| {
                    TargetError::Net(format!("unknown symbol `{s}` for field `{field}`"))
                })?;
                SignalValue::Num(raw as f64 * f.factor + f.shift)
            }
        };
        base.insert(field.clone(), value);
    }
    Ok(base)
}

fn message_name(message: &embrig_net::MessageDef) -> String {
    // The netmap has no reverse name lookup; this is only used in error text.
    format!("{}", message.dst)
}

fn load_netmap(path: &Path) -> Result<Netmap, TargetError> {
    let text = std::fs::read_to_string(path).map_err(|e| {
        TargetError::Net(format!("failed to read netmap `{}`: {e}", path.display()))
    })?;
    serde_saphyr::from_str(&text).map_err(|e| TargetError::Net(format!("invalid netmap: {e}")))
}

/// A [`TestTarget`] running against the deterministic virtual UDP network.
///
/// CAN steps (`send`, `poll`, `set_signal`, `add_fault`) are rejected with a
/// clear [`TargetError::UnsupportedOnTarget`]; UDP steps behave exactly like
/// their CAN counterparts.
pub struct UdpTarget {
    sim: UdpSim,
    netmap: Netmap,
    host: SocketAddr,
    network: Network,
    ecus: HashMap<String, usize>,
    config: VehicleConfig,
    net_config: NetworkConfig,
    netmap_path: PathBuf,
}

impl UdpTarget {
    /// Build a virtual UDP target from a vehicle config, network config and
    /// netmap file.
    pub fn new(
        config: &VehicleConfig,
        net_config: &NetworkConfig,
        netmap_path: &Path,
    ) -> Result<Self, TargetError> {
        let netmap = load_netmap(netmap_path)?;
        let built = build_udp_simulation(config, net_config, &netmap)?;
        Ok(Self {
            sim: built.sim,
            netmap,
            host: net_config.host,
            network: Network::default(),
            ecus: built.ecus.into_iter().collect(),
            config: config.clone(),
            net_config: net_config.clone(),
            netmap_path: netmap_path.to_path_buf(),
        })
    }

    /// Access the underlying simulation (for reports).
    pub fn sim(&self) -> &UdpSim {
        &self.sim
    }
}

impl TestTarget for UdpTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.sim.time()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        let netmap = load_netmap(&self.netmap_path)?;
        let built = build_udp_simulation(&self.config, &self.net_config, &netmap)?;
        self.netmap = netmap;
        self.sim = built.sim;
        self.ecus = built.ecus.into_iter().collect();
        Ok(())
    }

    fn set_signal(
        &mut self,
        _ecu: &str,
        _id: u32,
        _signal: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "set_signal is a CAN step; use set_field for UDP".into(),
        ))
    }

    fn add_fault(
        &mut self,
        _fault: embrig_core::fault::Fault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "add_fault is a CAN step; use add_fault_udp for UDP".into(),
        ))
    }

    async fn send(&mut self, _frame: embrig_core::frame::CanFrame) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "send is a CAN step; use send_udp for UDP".into(),
        ))
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        self.sim.run_for(duration);
        Ok(())
    }

    async fn poll(
        &mut self,
        _id: u32,
    ) -> Result<Option<embrig_core::frame::CanFrame>, TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "poll is a CAN step; use poll_udp for UDP".into(),
        ))
    }

    fn netmap(&self) -> Option<&Netmap> {
        Some(&self.netmap)
    }

    fn udp_host(&self) -> Result<SocketAddr, TargetError> {
        Ok(self.host)
    }

    async fn send_udp(&mut self, dg: UdpDatagram) -> Result<(), TargetError> {
        self.sim.inject(dg);
        Ok(())
    }

    async fn poll_udp(&mut self, dst: SocketAddr) -> Result<Option<UdpDatagram>, TargetError> {
        self.sim.run_for(POLL_US);
        Ok(self.sim.recorder().last_datagram(dst).cloned())
    }

    fn set_field(
        &mut self,
        ecu: &str,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), TargetError> {
        let index = *self
            .ecus
            .get(ecu)
            .ok_or_else(|| TargetError::UnknownEcu(ecu.to_string()))?;
        self.sim
            .set_field(index, message, field, value)
            .map_err(|e| TargetError::Net(e.to_string()))
    }

    fn add_fault_udp(
        &mut self,
        fault: UdpFault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        self.sim.add_fault(UdpFaultRule {
            fault,
            start: start.unwrap_or(self.sim.time()),
            duration,
        });
        Ok(())
    }
}

/// A [`TestTarget`] running the YAML suites against host-compiled firmware
/// (`udp-sil` ECUs) on the virtual UDP network.
///
/// Mirrors [`UdpTarget`], except field overrides on the firmware itself are
/// rejected (drive it via datagrams instead), and a firmware step exceeding
/// its budget fails the test.
pub struct UdpSutTarget {
    sim: UdpSim,
    netmap: Netmap,
    host: SocketAddr,
    network: Network,
    ecus: HashMap<String, usize>,
    sut: HashSet<String>,
    config: VehicleConfig,
    net_config: NetworkConfig,
    netmap_path: PathBuf,
    registry: UdpRegistry,
}

impl UdpSutTarget {
    /// Build a SIL UDP target. `registry` is owned so `reset` can re-invoke
    /// the firmware factories for a fresh simulation.
    pub fn new(
        config: &VehicleConfig,
        net_config: &NetworkConfig,
        netmap_path: &Path,
        registry: UdpRegistry,
    ) -> Result<Self, TargetError> {
        let netmap = load_netmap(netmap_path)?;
        let built = build_udp_simulation_with(config, net_config, &netmap, &registry)?;
        let sut = config
            .eth_ecus
            .iter()
            .filter(|e| e.kind == EthEcuKind::UdpSil)
            .map(|e| e.name.clone())
            .collect();
        Ok(Self {
            sim: built.sim,
            netmap,
            host: net_config.host,
            network: Network::default(),
            ecus: built.ecus.into_iter().collect(),
            sut,
            config: config.clone(),
            net_config: net_config.clone(),
            netmap_path: netmap_path.to_path_buf(),
            registry,
        })
    }

    /// The firmware registry (for diagnostics).
    pub fn registry(&self) -> &UdpRegistry {
        &self.registry
    }

    /// Run a firmware step, converting a firmware panic (e.g. a budget
    /// overrun) into a graceful test failure.
    fn run_sim<F, T>(&mut self, f: F) -> Result<T, TargetError>
    where
        F: FnOnce(&mut UdpSim) -> T,
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

impl TestTarget for UdpSutTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.sim.time()
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        let netmap = load_netmap(&self.netmap_path)?;
        let built =
            build_udp_simulation_with(&self.config, &self.net_config, &netmap, &self.registry)?;
        self.netmap = netmap;
        self.sim = built.sim;
        self.ecus = built.ecus.into_iter().collect();
        Ok(())
    }

    fn set_signal(
        &mut self,
        _ecu: &str,
        _id: u32,
        _signal: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "set_signal is a CAN step; use set_field for UDP".into(),
        ))
    }

    fn add_fault(
        &mut self,
        _fault: embrig_core::fault::Fault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "add_fault is a CAN step; use add_fault_udp for UDP".into(),
        ))
    }

    async fn send(&mut self, _frame: embrig_core::frame::CanFrame) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "send is a CAN step; use send_udp for UDP".into(),
        ))
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        self.run_sim(|sim| sim.run_for(duration))?;
        Ok(())
    }

    async fn poll(
        &mut self,
        _id: u32,
    ) -> Result<Option<embrig_core::frame::CanFrame>, TargetError> {
        Err(TargetError::UnsupportedOnTarget(
            "poll is a CAN step; use poll_udp for UDP".into(),
        ))
    }

    fn netmap(&self) -> Option<&Netmap> {
        Some(&self.netmap)
    }

    fn udp_host(&self) -> Result<SocketAddr, TargetError> {
        Ok(self.host)
    }

    async fn send_udp(&mut self, dg: UdpDatagram) -> Result<(), TargetError> {
        self.run_sim(|sim| sim.inject(dg))?;
        Ok(())
    }

    async fn poll_udp(&mut self, dst: SocketAddr) -> Result<Option<UdpDatagram>, TargetError> {
        self.run_sim(|sim| sim.run_for(POLL_US))?;
        Ok(self.sim.recorder().last_datagram(dst).cloned())
    }

    fn set_field(
        &mut self,
        ecu: &str,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), TargetError> {
        if self.sut.contains(ecu) {
            return Err(TargetError::UnsupportedOnSut(format!(
                "cannot override fields on firmware `{ecu}`; drive it via datagrams instead"
            )));
        }
        let index = *self
            .ecus
            .get(ecu)
            .ok_or_else(|| TargetError::UnknownEcu(ecu.to_string()))?;
        self.run_sim(|sim| sim.set_field(index, message, field, value))?
            .map_err(|e| TargetError::Net(e.to_string()))
    }

    fn add_fault_udp(
        &mut self,
        fault: UdpFault,
        start: Option<Timestamp>,
        duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        self.sim.add_fault(UdpFaultRule {
            fault,
            start: start.unwrap_or(self.sim.time()),
            duration,
        });
        Ok(())
    }
}

/// A [`TestTarget`] talking to a real Ethernet link through a bound UDP socket.
///
/// The socket is bound to the network's host endpoint. `send_udp` transmits a
/// datagram to its destination; `poll_udp` waits up to a poll interval and
/// returns the first datagram for the requested destination, queueing others
/// for later polls. `set_field` and faults are unsupported on a live link.
pub struct UdpHardwareTarget {
    socket: tokio::net::UdpSocket,
    host: SocketAddr,
    network: Network,
    netmap: Netmap,
    started: std::time::Instant,
    queue: VecDeque<UdpDatagram>,
}

impl UdpHardwareTarget {
    /// Bind a UDP socket to `host` for the network described by `netmap`.
    pub async fn new(host: SocketAddr, netmap: Netmap) -> Result<Self, TargetError> {
        let socket = tokio::net::UdpSocket::bind(host)
            .await
            .map_err(|e| TargetError::Net(format!("cannot bind UDP socket on {host}: {e}")))?;
        let bound = socket
            .local_addr()
            .map_err(|e| TargetError::Net(format!("cannot read bound UDP address: {e}")))?;
        Ok(Self {
            socket,
            host: bound,
            network: Network::default(),
            netmap,
            started: std::time::Instant::now(),
            queue: VecDeque::new(),
        })
    }
}

impl TestTarget for UdpHardwareTarget {
    fn network(&self) -> &Network {
        &self.network
    }

    fn elapsed_us(&self) -> Timestamp {
        self.started.elapsed().as_micros() as u64
    }

    fn reset(&mut self) -> Result<(), TargetError> {
        self.queue.clear();
        self.started = std::time::Instant::now();
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
        _fault: embrig_core::fault::Fault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "faults require the virtual router".into(),
        ))
    }

    async fn send(&mut self, _frame: embrig_core::frame::CanFrame) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "CAN frames cannot be sent on a UDP link".into(),
        ))
    }

    async fn wait(&mut self, duration: Timestamp) -> Result<(), TargetError> {
        tokio::time::sleep(std::time::Duration::from_micros(duration)).await;
        Ok(())
    }

    async fn poll(
        &mut self,
        _id: u32,
    ) -> Result<Option<embrig_core::frame::CanFrame>, TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "CAN frames cannot be received on a UDP link".into(),
        ))
    }

    fn netmap(&self) -> Option<&Netmap> {
        Some(&self.netmap)
    }

    fn udp_host(&self) -> Result<SocketAddr, TargetError> {
        Ok(self.host)
    }

    async fn send_udp(&mut self, dg: UdpDatagram) -> Result<(), TargetError> {
        self.socket
            .send_to(&dg.payload, dg.dst)
            .await
            .map_err(|e| TargetError::Net(format!("cannot send to {}: {e}", dg.dst)))?;
        Ok(())
    }

    async fn poll_udp(&mut self, dst: SocketAddr) -> Result<Option<UdpDatagram>, TargetError> {
        let mut buf = [0u8; 65535];
        let deadline = std::time::Instant::now() + std::time::Duration::from_micros(POLL_US);
        loop {
            if let Some(pos) = self.queue.iter().position(|d| d.dst == dst) {
                return Ok(Some(self.queue.remove(pos).expect("position found")));
            }
            let now = std::time::Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let remaining = deadline.saturating_duration_since(now);
            match tokio::time::timeout(remaining, self.socket.recv_from(&mut buf)).await {
                Ok(Ok((n, src))) => {
                    self.queue
                        .push_back(UdpDatagram::new(src, self.host, buf[..n].to_vec()));
                }
                Ok(Err(e)) => return Err(TargetError::Net(format!("cannot receive: {e}"))),
                Err(_) => return Ok(None),
            }
        }
    }

    fn set_field(
        &mut self,
        _ecu: &str,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "set_field cannot inject into a live network".into(),
        ))
    }

    fn add_fault_udp(
        &mut self,
        _fault: UdpFault,
        _start: Option<Timestamp>,
        _duration: Option<Timestamp>,
    ) -> Result<(), TargetError> {
        Err(TargetError::UnsupportedOnHardware(
            "faults require the virtual router".into(),
        ))
    }
}

/// Run the YAML test files against the virtual UDP network.
///
/// Convenience wrapper over [`UdpTarget`] + [`run_suite`]; use the building
/// blocks directly if you need to reuse a target.
pub fn udp_run(
    config: &VehicleConfig,
    net_config: &NetworkConfig,
    netmap_path: &Path,
    files: &[PathBuf],
) -> Result<SuiteResult, UdpError> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut target = UdpTarget::new(config, net_config, netmap_path)?;
        run_suite(&mut target, files, "udp")
            .await
            .map_err(UdpError::from)
    })
}

/// Run the YAML test files against the firmware in `registry` on the virtual
/// UDP network.
pub fn udp_run_with_firmware(
    config: &VehicleConfig,
    net_config: &NetworkConfig,
    netmap_path: &Path,
    registry: UdpRegistry,
    files: &[PathBuf],
) -> Result<SuiteResult, UdpError> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut target = UdpSutTarget::new(config, net_config, netmap_path, registry)?;
        run_suite(&mut target, files, "udp-sil")
            .await
            .map_err(UdpError::from)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_net::UdpEcuError;
    use std::io::Write;

    const NETMAP: &str = r#"
messages:
  DriveCommand:
    dst: 192.168.1.30:5000
    length: 8
    fields:
      forward: { offset: 0, type: f32le }
      estop:   { offset: 4, type: bool }
  MotionState:
    dst: 192.168.1.10:5000
    length: 8
    fields:
      speed: { offset: 0, type: f32le }
      state:
        offset: 4
        type: u8
        values:
          0: STOPPED
          1: DRIVING
          2: EMERGENCY
"#;

    fn tmp_file(name: &str, content: &str) -> std::path::PathBuf {
        use std::sync::atomic::{AtomicU64, Ordering};
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let dir = std::env::temp_dir().join(format!(
            "embrig-udp-{}-{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn netmap_file() -> std::path::PathBuf {
        tmp_file("netmap.yaml", NETMAP)
    }

    fn vehicle() -> VehicleConfig {
        serde_saphyr::from_str(
            r#"
name: rover
step_us: 1000
eth_ecus:
  - name: joystick
    type: udp-config
    address: 192.168.1.20:6000
    message: DriveCommand
    period_us: 20000
  - name: motion
    type: udp-config
    address: 192.168.1.30:5000
    message: MotionState
    period_us: 50000
    fields:
      speed: 0.0
      state: STOPPED
networks:
  - name: eth
    type: udp
    host: 192.168.1.10:5000
    netmap: netmap.yaml
interfaces:
  - name: udp
    type: udp
"#,
        )
        .unwrap()
    }

    fn net_config() -> NetworkConfig {
        vehicle()
            .networks
            .iter()
            .find(|n| n.kind == "udp")
            .unwrap()
            .clone()
    }

    fn suite(name: &str, steps: &str) -> std::path::PathBuf {
        tmp_file(name, &format!("name: {name}\ntimeout: 5s\nsteps:\n{steps}"))
    }

    const HOST: &str = "192.168.1.10:5000";
    const MOTION: &str = "192.168.1.30:5000";

    #[tokio::test]
    async fn virtual_target_reports_and_injects() {
        let mut target = UdpTarget::new(&vehicle(), &net_config(), &netmap_file()).unwrap();
        let host: SocketAddr = HOST.parse().unwrap();
        let motion: SocketAddr = MOTION.parse().unwrap();
        assert_eq!(target.elapsed_us(), 0);

        // Telemetry is periodic: the config motion ECU transmits every 50ms.
        target.wait(110_000).await.unwrap();
        let telemetry = target.poll_udp(host).await.unwrap().unwrap();
        assert_eq!(telemetry.dst, host);
        let speed = target
            .netmap()
            .unwrap()
            .message("MotionState")
            .unwrap()
            .decode_field(&telemetry.payload, "speed")
            .unwrap();
        assert_eq!(speed.value, 0.0);

        // Inject a drive command from the host to the motion ECU.
        let message = target
            .netmap()
            .unwrap()
            .message("DriveCommand")
            .unwrap()
            .clone();
        let payload = message
            .encode_fields(&[("forward", SignalValue::Num(2.0))])
            .unwrap();
        target
            .send_udp(UdpDatagram::new(host, motion, payload))
            .await
            .unwrap();
        assert!(target.poll_udp(motion).await.unwrap().is_some());
    }

    #[tokio::test]
    async fn virtual_target_overrides_config_ecu_fields() {
        let mut target = UdpTarget::new(&vehicle(), &net_config(), &netmap_file()).unwrap();
        let host: SocketAddr = HOST.parse().unwrap();
        target
            .set_field("motion", "MotionState", "speed", SignalValue::Num(3.5))
            .unwrap();
        target.wait(60_000).await.unwrap();
        let telemetry = target.poll_udp(host).await.unwrap().unwrap();
        let speed = target
            .netmap()
            .unwrap()
            .message("MotionState")
            .unwrap()
            .decode_field(&telemetry.payload, "speed")
            .unwrap();
        assert!((speed.value - 3.5).abs() < 1e-6);
    }

    #[tokio::test]
    async fn can_steps_are_rejected_on_udp_target() {
        let mut target = UdpTarget::new(&vehicle(), &net_config(), &netmap_file()).unwrap();
        let err = target.set_signal("motion", 0x100, "x", SignalValue::Num(1.0));
        assert!(matches!(err, Err(TargetError::UnsupportedOnTarget(_))));
        let err = target
            .send(embrig_core::frame::CanFrame::new(0x100, vec![0; 8]).unwrap())
            .await
            .unwrap_err();
        assert!(matches!(err, TargetError::UnsupportedOnTarget(_)));
    }

    #[test]
    fn udp_suite_passes() {
        let files = vec![
            suite(
                "telemetry_reports_speed.yaml",
                "  - wait: { time: 60ms }\n  - set_field: { ecu: motion, message: MotionState, field: speed, value: 2.0 }\n  - expect_udp: { message: MotionState, field: speed, equals: 2.0, within: 1s }\n",
            ),
            suite(
                "drive_command_observed.yaml",
                "  - send_udp: { message: DriveCommand, fields: { forward: 2.0 } }\n  - expect_udp: { message: DriveCommand, present: true, within: 1s }\n",
            ),
        ];
        let result = udp_run(&vehicle(), &net_config(), &netmap_file(), &files).unwrap();
        assert_eq!(result.failed(), 0, "failures: {:?}", result.tests);
    }

    #[test]
    fn udp_fault_drops_telemetry() {
        let files = vec![suite(
            "drop_motion_state.yaml",
            "  - fault_udp: { type: drop, message: MotionState, duration: 500ms }\n  - wait: { time: 100ms }\n  - expect_udp: { message: MotionState, present: false, within: 100ms }\n",
        )];
        let result = udp_run(&vehicle(), &net_config(), &netmap_file(), &files).unwrap();
        assert_eq!(result.failed(), 0, "failures: {:?}", result.tests);
    }

    // --- firmware (SIL) path ---

    /// Firmware that decides speed/state from received drive commands.
    struct TestFirmware {
        name: String,
        netmap: Netmap,
        src: SocketAddr,
        forward: f64,
        estop: bool,
        next_tx: Timestamp,
    }

    impl TestFirmware {
        fn new(name: &str, netmap: Netmap, src: SocketAddr) -> Self {
            Self {
                name: name.to_string(),
                netmap,
                src,
                forward: 0.0,
                estop: false,
                next_tx: 0,
            }
        }
    }

    impl embrig_net::UdpEcu for TestFirmware {
        fn name(&self) -> &str {
            &self.name
        }

        fn on_datagram(&mut self, dg: &UdpDatagram, _time: Timestamp) {
            if let Some(m) = self.netmap.message("DriveCommand") {
                if dg.payload.len() >= m.length {
                    self.forward = m
                        .decode_field(&dg.payload, "forward")
                        .map(|d| d.value)
                        .unwrap_or(self.forward);
                    self.estop = m
                        .decode_field(&dg.payload, "estop")
                        .map(|d| d.value > 0.5)
                        .unwrap_or(self.estop);
                }
            }
        }

        fn update(&mut self, time: Timestamp, out: &mut Vec<UdpDatagram>) {
            if time >= self.next_tx {
                let m = self.netmap.message("MotionState").unwrap().clone();
                let (state, speed) = if self.estop {
                    (2, 0.0)
                } else if self.forward > 0.0 {
                    (1, self.forward.clamp(0.0, 5.0))
                } else {
                    (0, 0.0)
                };
                if let Ok(payload) = m.encode_fields(&[
                    ("speed", SignalValue::Num(speed)),
                    ("state", SignalValue::Num(state as f64)),
                ]) {
                    out.push(UdpDatagram::new(self.src, m.dst, payload));
                }
                self.next_tx = time + 50_000;
            }
        }
    }

    fn sil_vehicle() -> VehicleConfig {
        let mut cfg = vehicle();
        // The motion node is the system under test.
        cfg.eth_ecus[1].kind = EthEcuKind::UdpSil;
        cfg
    }

    fn firmware_registry() -> UdpRegistry {
        let motion: SocketAddr = MOTION.parse().unwrap();
        let netmap: Netmap = serde_saphyr::from_str(NETMAP).unwrap();
        let mut registry = UdpRegistry::new();
        registry.register(
            "motion",
            move |name: &str, _budget: u64| -> Result<Box<dyn embrig_net::UdpEcu>, UdpEcuError> {
                Ok(Box::new(TestFirmware::new(name, netmap.clone(), motion)))
            },
        );
        registry
    }

    #[test]
    fn sil_suite_drives_firmware_over_udp() {
        let files = vec![
            suite(
                "drive_command_moves_rover.yaml",
                "  - send_udp: { message: DriveCommand, fields: { forward: 2.0 } }\n  - expect_udp: { message: MotionState, field: speed, greater_than: 1.5, within: 1s }\n  - expect_udp: { message: MotionState, field: state, equals: \"DRIVING\" }\n",
            ),
            suite(
                "estop_stops_rover.yaml",
                "  - send_udp: { message: DriveCommand, fields: { estop: true } }\n  - expect_udp: { message: MotionState, field: state, equals: \"EMERGENCY\", within: 1s }\n  - expect_udp: { message: MotionState, field: speed, less_than: 0.5 }\n",
            ),
        ];
        let result = udp_run_with_firmware(
            &sil_vehicle(),
            &net_config(),
            &netmap_file(),
            firmware_registry(),
            &files,
        )
        .unwrap();
        assert_eq!(result.failed(), 0, "failures: {:?}", result.tests);
    }

    #[tokio::test]
    async fn sut_field_override_is_rejected() {
        let mut target = UdpSutTarget::new(
            &sil_vehicle(),
            &net_config(),
            &netmap_file(),
            firmware_registry(),
        )
        .unwrap();
        let err = target.set_field("motion", "MotionState", "speed", SignalValue::Num(1.0));
        assert!(matches!(err, Err(TargetError::UnsupportedOnSut(_))));
        // Config ECUs still accept overrides.
        target
            .set_field("joystick", "DriveCommand", "forward", SignalValue::Num(1.0))
            .unwrap();
    }

    #[test]
    fn udp_sil_without_firmware_fails_with_a_clear_error() {
        let err = match UdpSutTarget::new(
            &sil_vehicle(),
            &net_config(),
            &netmap_file(),
            UdpRegistry::new(),
        ) {
            Ok(_) => panic!("expected a clear startup error"),
            Err(e) => e,
        };
        let message = err.to_string();
        assert!(
            message.contains("no firmware registered for SIL ECU `motion`"),
            "got: {message}"
        );
    }

    // --- hardware loopback ---

    #[tokio::test]
    async fn udp_loopback_hardware_target() {
        let netmap: Netmap = serde_saphyr::from_str(NETMAP).unwrap();
        let host: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let mut target = UdpHardwareTarget::new(host, netmap).await.unwrap();
        let actual_host = target.udp_host().unwrap();
        let client = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let client_addr = client.local_addr().unwrap();

        // Client -> host.
        let payload = vec![1, 2, 3, 4];
        client.send_to(&payload, actual_host).await.unwrap();
        let dg = target
            .poll_udp(actual_host)
            .await
            .unwrap()
            .expect("datagram from client");
        assert_eq!(dg.payload, payload);
        assert_eq!(dg.src, client_addr);

        // Host -> client.
        let reply = vec![5, 6];
        target
            .send_udp(UdpDatagram::new(actual_host, client_addr, reply.clone()))
            .await
            .unwrap();
        let mut buf = [0u8; 64];
        let (n, src) = client.recv_from(&mut buf).await.unwrap();
        assert_eq!(&buf[..n], &reply[..]);
        assert_eq!(src, actual_host);
    }
}
