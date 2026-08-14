//! Interface protocols: map an interface `kind` to a target builder.
//!
//! A [`Protocol`] knows the `kind` it serves (`virtual`, `udp`, `socketcan`,
//! `restbus`) and builds a boxed, object-safe [`DynTestTarget`] from a
//! [`ProtocolInput`].
//! The [`ProtocolRegistry`] holds the known protocols, so a `kind` picked from
//! YAML or the CLI can be resolved to a target without the caller matching on
//! concrete types. Crates that host extra transports (e.g. `embrig-sil`)
//! register their own protocols.

use std::collections::HashMap;
use std::path::Path;

use embrig_models::VehicleConfig;

use crate::target::{DynTestTarget, TargetError};
use crate::udp::UdpTarget;
use crate::VirtualTarget;

/// Everything a protocol needs to build a target.
///
/// `dbc_path` is the already-resolved DBC file (empty when the vehicle is
/// pure-Ethernet); `vehicle_dir` is the directory containing `vehicle.yaml`,
/// used to resolve relative netmap paths; `interface` is the raw interface
/// name given on the CLI, if any (the socketcan protocol uses it to pick the
/// kernel interface).
pub struct ProtocolInput<'a> {
    pub config: &'a VehicleConfig,
    pub dbc_path: &'a Path,
    pub vehicle_dir: &'a Path,
    pub interface: Option<&'a str>,
}

/// A transport protocol: identified by `kind` and able to build a target.
pub trait Protocol {
    fn kind(&self) -> &str;
    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError>;
}

/// A [`Protocol`] backed by a closure, for ad-hoc registration.
struct FnProtocol<F> {
    kind: &'static str,
    build_fn: F,
}

impl<F> FnProtocol<F> {
    fn new(kind: &'static str, build_fn: F) -> Self {
        Self { kind, build_fn }
    }
}

impl<F> Protocol for FnProtocol<F>
where
    F: for<'a> Fn(&ProtocolInput<'a>) -> Result<Box<dyn DynTestTarget>, TargetError>,
{
    fn kind(&self) -> &str {
        self.kind
    }

    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        (self.build_fn)(input)
    }
}

/// The deterministic virtual CAN simulation.
pub struct VirtualProtocol;

impl Protocol for VirtualProtocol {
    fn kind(&self) -> &str {
        "virtual"
    }

    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        VirtualTarget::new(input.config, input.dbc_path)
            .map(|target| Box::new(target) as Box<dyn DynTestTarget>)
            .map_err(|e| {
                TargetError::Build(format!(
                    "cannot build virtual simulation from `{}`: {e}",
                    input.dbc_path.display()
                ))
            })
    }
}

/// The deterministic virtual UDP network.
pub struct UdpProtocol;

impl Protocol for UdpProtocol {
    fn kind(&self) -> &str {
        "udp"
    }

    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        let net_config = input
            .config
            .networks
            .iter()
            .find(|n| n.kind == "udp")
            .ok_or_else(|| {
                TargetError::Build("vehicle.yaml has no network of type `udp`".into())
            })?;
        let netmap_path = input.vehicle_dir.join(&net_config.netmap);
        if !netmap_path.exists() {
            return Err(TargetError::Build(format!(
                "netmap file `{}` not found",
                netmap_path.display()
            )));
        }
        UdpTarget::new(input.config, net_config, &netmap_path)
            .map(|target| Box::new(target) as Box<dyn DynTestTarget>)
            .map_err(|e| {
                TargetError::Build(format!(
                    "cannot build UDP simulation from `{}`: {e}",
                    netmap_path.display()
                ))
            })
    }
}

/// Resolve the kernel interface name for a socket-backed protocol: a declared
/// interface name yields its `interface` field, anything else is treated as a
/// raw kernel interface name, defaulting to `vcan0`.
#[cfg(feature = "socketcan")]
fn socketcan_interface(input: &ProtocolInput<'_>) -> String {
    match input.interface {
        Some(name) => input
            .config
            .interfaces
            .iter()
            .find(|i| i.name == name)
            .and_then(|i| i.interface.clone())
            .unwrap_or_else(|| name.to_string()),
        None => "vcan0".to_string(),
    }
}

/// A real SocketCAN interface.
#[cfg(feature = "socketcan")]
pub struct SocketcanProtocol;

#[cfg(feature = "socketcan")]
impl Protocol for SocketcanProtocol {
    fn kind(&self) -> &str {
        "socketcan"
    }

    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        let iface_name = socketcan_interface(input);
        let text = std::fs::read_to_string(input.dbc_path).map_err(|e| {
            TargetError::Build(format!("cannot read `{}`: {e}", input.dbc_path.display()))
        })?;
        let network = embrig_dbc::parse(&text).map_err(|e| {
            TargetError::Build(format!("invalid DBC `{}`: {e}", input.dbc_path.display()))
        })?;
        crate::target::HardwareTarget::new(&iface_name, network)
            .map(|target| Box::new(target) as Box<dyn DynTestTarget>)
            .map_err(|e| {
                TargetError::Build(format!("cannot open CAN interface `{iface_name}`: {e}"))
            })
    }
}

/// A real SocketCAN interface bridged to the simulated rest bus.
///
/// The simulated nodes from `vehicle.yaml` run alongside the real ECU on the
/// bus, so `set_signal` and faults work on HIL.
#[cfg(feature = "socketcan")]
pub struct RestBusProtocol;

#[cfg(feature = "socketcan")]
impl Protocol for RestBusProtocol {
    fn kind(&self) -> &str {
        "restbus"
    }

    fn build(&self, input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        let iface_name = socketcan_interface(input);
        crate::target::RestBusTarget::new(input.config, input.dbc_path, &iface_name)
            .map(|target| Box::new(target) as Box<dyn DynTestTarget>)
            .map_err(|e| TargetError::Build(format!("cannot open rest bus `{iface_name}`: {e}")))
    }
}

/// Registered placeholder so `--interface socketcan` still explains itself on
/// builds without the feature instead of reporting an unknown protocol.
#[cfg(not(feature = "socketcan"))]
pub struct SocketcanProtocol;

#[cfg(not(feature = "socketcan"))]
impl Protocol for SocketcanProtocol {
    fn kind(&self) -> &str {
        "socketcan"
    }

    fn build(&self, _input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        Err(TargetError::Build(
            "this build has no socketcan support; rebuild with `--features socketcan`".into(),
        ))
    }
}

/// Registered placeholder so `--interface restbus` still explains itself on
/// builds without the feature instead of reporting an unknown protocol.
#[cfg(not(feature = "socketcan"))]
pub struct RestBusProtocol;

#[cfg(not(feature = "socketcan"))]
impl Protocol for RestBusProtocol {
    fn kind(&self) -> &str {
        "restbus"
    }

    fn build(&self, _input: &ProtocolInput<'_>) -> Result<Box<dyn DynTestTarget>, TargetError> {
        Err(TargetError::Build(
            "this build has no socketcan support; rebuild with `--features socketcan`".into(),
        ))
    }
}

/// Registry of [`Protocol`]s keyed by interface `kind`.
///
/// [`Default`] ships the built-in protocols: `virtual`, `udp` and (when the
/// `socketcan` feature is on) `socketcan`. Other crates add transports with
/// [`ProtocolRegistry::register`].
pub struct ProtocolRegistry {
    protocols: HashMap<String, Box<dyn Protocol>>,
}

impl ProtocolRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self {
            protocols: HashMap::new(),
        }
    }

    /// Register a protocol under [`Protocol::kind`]. Registering a second
    /// protocol with the same kind replaces the first.
    pub fn register<P: Protocol + 'static>(&mut self, protocol: P) {
        self.protocols
            .insert(protocol.kind().to_string(), Box::new(protocol));
    }

    /// Register a protocol from a builder closure, for ad-hoc transports that
    /// do not need their own [`Protocol`] type.
    pub fn register_fn<F>(&mut self, kind: &'static str, build_fn: F)
    where
        F: for<'a> Fn(&ProtocolInput<'a>) -> Result<Box<dyn DynTestTarget>, TargetError> + 'static,
    {
        self.register(FnProtocol::new(kind, build_fn));
    }

    /// Build a target for `kind`. Unknown kinds are a build error.
    pub fn build(
        &self,
        kind: &str,
        input: &ProtocolInput<'_>,
    ) -> Result<Box<dyn DynTestTarget>, TargetError> {
        let protocol = self.protocols.get(kind).ok_or_else(|| {
            TargetError::Build(format!(
                "no protocol registered for interface type `{kind}`"
            ))
        })?;
        protocol.build(input)
    }
}

impl Default for ProtocolRegistry {
    fn default() -> Self {
        let mut registry = ProtocolRegistry::new();
        registry.register(VirtualProtocol);
        registry.register(UdpProtocol);
        registry.register(SocketcanProtocol);
        registry.register(RestBusProtocol);
        registry
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::path::PathBuf;

    use embrig_core::frame::CanFrame;
    use embrig_models::load_vehicle_config;

    fn rover() -> (VehicleConfig, PathBuf) {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rover");
        load_vehicle_config(&root.join("vehicle.yaml"))
            .unwrap_or_else(|e| panic!("load rover vehicle: {e}"))
    }

    fn tmp_file(name: &str, content: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("embrig-test-protocol-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn demo_config(tag: &str) -> (VehicleConfig, PathBuf) {
        let config: VehicleConfig = serde_saphyr::from_str(
            &std::fs::read_to_string(tmp_file(
                &format!("{tag}-vehicle.yaml"),
                &format!(
                    r#"
name: demo
dbc: {tag}.dbc
step_us: 1000
ecus:
  - name: battery
    type: config
    message: BatteryStatus
    period_us: 100000
    signals:
      voltage: 400.0
interfaces:
  - name: virtual
    type: virtual
"#
                ),
            ))
            .unwrap(),
        )
        .unwrap();
        let dbc = tmp_file(
            &format!("{tag}.dbc"),
            r#"VERSION ""

NS_ :

BS_:

BU_: engine

BO_ 256 BatteryStatus: 8 engine
 SG_ voltage : 0|16@1+ (0.1,0) [0|600] "V" engine
"#,
        );
        (config, dbc)
    }

    fn build_ok(
        registry: &ProtocolRegistry,
        kind: &str,
        input: &ProtocolInput<'_>,
    ) -> Box<dyn DynTestTarget> {
        match registry.build(kind, input) {
            Ok(target) => target,
            Err(e) => panic!("build `{kind}`: {e}"),
        }
    }

    fn build_err(
        registry: &ProtocolRegistry,
        kind: &str,
        input: &ProtocolInput<'_>,
    ) -> TargetError {
        match registry.build(kind, input) {
            Ok(_) => panic!("build `{kind}` unexpectedly succeeded"),
            Err(e) => e,
        }
    }

    #[test]
    fn default_registry_builds_each_builtin_protocol() {
        let registry = ProtocolRegistry::default();

        let (config, dbc_path) = rover();
        let vehicle_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rover");
        let udp_input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: &vehicle_dir,
            interface: None,
        };
        let udp = build_ok(&registry, "udp", &udp_input);
        assert!(udp.netmap().is_some());
        drop(udp);

        let (config, dbc_path) = demo_config("proto-default");
        let virtual_input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: Path::new("."),
            interface: None,
        };
        let built = build_ok(&registry, "virtual", &virtual_input);
        assert!(built.netmap().is_none());
    }

    #[test]
    fn register_fn_adds_a_custom_protocol() {
        let mut registry = ProtocolRegistry::new();
        registry.register_fn("custom", |input| {
            crate::VirtualTarget::new(input.config, input.dbc_path)
                .map(|t| Box::new(t) as Box<dyn DynTestTarget>)
        });

        let (config, dbc_path) = demo_config("proto-regfn");
        let input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: Path::new("."),
            interface: None,
        };
        let target = build_ok(&registry, "custom", &input);
        assert!(target.netmap().is_none());
        build_err(&registry, "virtual", &input);
    }

    #[test]
    fn unknown_kind_is_a_build_error() {
        let registry = ProtocolRegistry::default();
        let (config, dbc_path) = demo_config("proto-unknown");
        let input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: Path::new("."),
            interface: None,
        };
        let err = build_err(&registry, "nonsense", &input);
        match err {
            TargetError::Build(msg) => assert!(msg.contains("`nonsense`"), "got: {msg}"),
            other => panic!("expected Build error, got {other:?}"),
        }
    }

    #[test]
    fn udp_protocol_rejects_missing_network() {
        let registry = ProtocolRegistry::default();
        let (config, dbc_path) = demo_config("proto-nonet");
        let input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: Path::new("."),
            interface: None,
        };
        let err = build_err(&registry, "udp", &input);
        match err {
            TargetError::Build(msg) => {
                assert!(msg.contains("no network of type `udp`"), "got: {msg}")
            }
            other => panic!("expected Build error, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn registry_target_is_driveable_through_dyn() {
        let registry = ProtocolRegistry::default();
        let (config, dbc_path) = demo_config("proto-dyn");
        let input = ProtocolInput {
            config: &config,
            dbc_path: &dbc_path,
            vehicle_dir: Path::new("."),
            interface: None,
        };
        let mut target = build_ok(&registry, "virtual", &input);
        target.wait(50_000).await.unwrap();
        assert_eq!(target.elapsed_us(), 50_000);
        target
            .send(CanFrame::new(0x100, vec![0xFA; 8]).unwrap())
            .await
            .unwrap();
    }
}
