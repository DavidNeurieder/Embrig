//! Vehicle models: YAML configuration, config-driven ECUs and the reference
//! EV-powertrain vECUs (charger, VCU, motor), plus the [`build_simulation`]
//! entry point that turns a [`VehicleConfig`] into a runnable [`Simulation`].
//! `type: sil` ECU nodes are bound to host-compiled firmware through the
//! [`EcuFactory`] hook ([`build_simulation_indexed_with`]).

pub mod config;
pub mod ecus;

pub use config::{
    literal_to_physical, ConfigEcu, EcuConfig, EcuKind, EthEcuConfig, EthEcuKind, InterfaceConfig,
    NetworkConfig, SignalLiteral, VehicleConfig,
};
pub use ecus::{
    Charger, Motor, VehicleController, ID_BATTERY, ID_BRAKE, ID_CHARGE_REQUEST, ID_CHARGE_STATUS,
    ID_DRIVER, ID_MOTOR_ENABLE, ID_MOTOR_STATUS,
};

use std::fs;
use std::path::Path;

use embrig_core::{Ecu, Simulation};
use thiserror::Error;

/// Errors while assembling a simulation from a vehicle config.
#[derive(Debug, Error)]
pub enum ModelError {
    #[error("failed to read vehicle config `{path}`: {source}")]
    ReadVehicle {
        path: String,
        source: std::io::Error,
    },
    #[error("invalid vehicle config `{path}`: {message}")]
    ParseVehicle { path: String, message: String },
    #[error("failed to read DBC file `{path}`: {source}")]
    ReadDbc {
        path: String,
        source: std::io::Error,
    },
    #[error("failed to parse DBC file: {0}")]
    ParseDbc(embrig_dbc::ParseError),
    #[error("config ECU `{0}` has no `message` name")]
    MissingMessage(String),
    #[error("message `{name}` not found in DBC file")]
    UnknownMessageName { name: String },
    #[error("message id 0x{id:03X} (needed by ECU `{ecu}`) not found in DBC file")]
    MissingId { id: u32, ecu: String },
    #[error("ECU `{ecu}`: {message}")]
    Ecu {
        ecu: String,
        message: embrig_core::EcuError,
    },
}

/// A built virtual simulation plus the ECU name → index map needed to
/// override signals by name at runtime.
pub struct BuiltSimulation {
    pub sim: Simulation,
    pub ecus: Vec<(String, usize)>,
}

/// Creates the [`Ecu`] implementation for a `type: sil` ECU node. Used by
/// software-in-the-loop tooling so firmware (host-compiled) is the code, not
/// the config. `step_budget_us` is the wall-clock budget for one simulated
/// step (the firmware must not exceed it).
pub trait EcuFactory: Send + Sync {
    fn create(
        &self,
        name: &str,
        step_budget_us: u64,
    ) -> Result<Box<dyn Ecu>, embrig_core::EcuError>;
}

/// A factory registry with no firmware: instantiating any `type: sil` ECU
/// fails with [`ModelError::NoSilFirmware`].
#[derive(Default)]
pub struct NoSils;

impl EcuFactory for NoSils {
    fn create(
        &self,
        name: &str,
        _step_budget_us: u64,
    ) -> Result<Box<dyn Ecu>, embrig_core::EcuError> {
        Err(embrig_core::EcuError::NotRegistered(name.to_string()))
    }
}

/// Lets callers register a firmware factory as a plain closure, e.g.
/// `registry.register("controller", |_name, _budget| Ok(Box::new(Firmware::new())))`.
impl<F> EcuFactory for F
where
    F: Fn(&str, u64) -> Result<Box<dyn Ecu>, embrig_core::EcuError> + Send + Sync + 'static,
{
    fn create(
        &self,
        name: &str,
        step_budget_us: u64,
    ) -> Result<Box<dyn Ecu>, embrig_core::EcuError> {
        self(name, step_budget_us)
    }
}

/// Load a `vehicle.yaml` file and resolve its DBC path relative to it.
pub fn load_vehicle_config(path: &Path) -> Result<(VehicleConfig, std::path::PathBuf), ModelError> {
    let text = fs::read_to_string(path).map_err(|source| ModelError::ReadVehicle {
        path: path.display().to_string(),
        source,
    })?;
    let config: VehicleConfig =
        serde_saphyr::from_str(&text).map_err(|message| ModelError::ParseVehicle {
            path: path.display().to_string(),
            message: message.to_string(),
        })?;
    let dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let dbc_path = dir.join(&config.dbc);
    Ok((config, dbc_path))
}

/// Build a virtual simulation from a vehicle config and its DBC file.
///
/// ECUs are attached in the order listed in `config.ecus`, which keeps the
/// simulation deterministic. The reference vECUs (`charger`, `vcu`, `motor`)
/// use the fixed message ids of the powertrain DBC.
pub fn build_simulation(config: &VehicleConfig, dbc_path: &Path) -> Result<Simulation, ModelError> {
    Ok(build_simulation_indexed(config, dbc_path)?.sim)
}

/// Like [`build_simulation`] but also returns the ECU name → index map.
pub fn build_simulation_indexed(
    config: &VehicleConfig,
    dbc_path: &Path,
) -> Result<BuiltSimulation, ModelError> {
    build_simulation_indexed_with(config, dbc_path, NoSils)
}

/// Like [`build_simulation_indexed`] but lets callers provide the firmware
/// factories for `type: sil` ECU nodes (see [`EcuFactory`]).
pub fn build_simulation_indexed_with(
    config: &VehicleConfig,
    dbc_path: &Path,
    factories: impl EcuFactory,
) -> Result<BuiltSimulation, ModelError> {
    let text = fs::read_to_string(dbc_path).map_err(|source| ModelError::ReadDbc {
        path: dbc_path.display().to_string(),
        source,
    })?;
    let network = embrig_dbc::parse(&text).map_err(ModelError::ParseDbc)?;

    let mut sim = Simulation::new(config.step_us);
    let mut ecus: Vec<(String, usize)> = Vec::new();

    for ecu_cfg in &config.ecus {
        let index = match ecu_cfg.kind {
            EcuKind::Config => {
                let msg_name = ecu_cfg
                    .message
                    .as_ref()
                    .ok_or_else(|| ModelError::MissingMessage(ecu_cfg.name.clone()))?;
                let message = network
                    .messages
                    .values()
                    .find(|m| &m.name == msg_name)
                    .ok_or_else(|| ModelError::UnknownMessageName {
                        name: msg_name.clone(),
                    })?
                    .clone();
                let ecu = ConfigEcu::new(
                    ecu_cfg.name.clone(),
                    message,
                    ecu_cfg.period_us,
                    &ecu_cfg.signals,
                )
                .map_err(|message| ModelError::Ecu {
                    ecu: ecu_cfg.name.clone(),
                    message,
                })?;
                sim.attach(Box::new(ecu), &ecu_cfg.listen)
            }
            EcuKind::Charger => {
                let req = network
                    .message(ecus::ID_CHARGE_REQUEST)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_CHARGE_REQUEST,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let battery = network
                    .message(ecus::ID_BATTERY)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_BATTERY,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let status = network
                    .message(ecus::ID_CHARGE_STATUS)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_CHARGE_STATUS,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let ecu = Charger::new(
                    ecu_cfg.name.clone(),
                    req,
                    battery,
                    status,
                    ecu_cfg.period_us,
                );
                sim.attach(Box::new(ecu), &[ecus::ID_CHARGE_REQUEST, ecus::ID_BATTERY])
            }
            EcuKind::Vcu => {
                let battery = network
                    .message(ecus::ID_BATTERY)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_BATTERY,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let brake = network
                    .message(ecus::ID_BRAKE)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_BRAKE,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let driver = network
                    .message(ecus::ID_DRIVER)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_DRIVER,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let charger = network
                    .message(ecus::ID_CHARGE_STATUS)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_CHARGE_STATUS,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let enable = network
                    .message(ecus::ID_MOTOR_ENABLE)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_MOTOR_ENABLE,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let ecu = VehicleController::new(
                    ecu_cfg.name.clone(),
                    battery,
                    brake,
                    driver,
                    charger,
                    enable,
                    ecu_cfg.period_us,
                );
                sim.attach(
                    Box::new(ecu),
                    &[
                        ecus::ID_BATTERY,
                        ecus::ID_BRAKE,
                        ecus::ID_DRIVER,
                        ecus::ID_CHARGE_STATUS,
                    ],
                )
            }
            EcuKind::Motor => {
                let enable = network
                    .message(ecus::ID_MOTOR_ENABLE)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_MOTOR_ENABLE,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let status = network
                    .message(ecus::ID_MOTOR_STATUS)
                    .ok_or(ModelError::MissingId {
                        id: ecus::ID_MOTOR_STATUS,
                        ecu: ecu_cfg.name.clone(),
                    })?
                    .clone();
                let ecu = Motor::new(ecu_cfg.name.clone(), enable, status, ecu_cfg.period_us);
                sim.attach(Box::new(ecu), &[ecus::ID_MOTOR_ENABLE])
            }
            EcuKind::Sil => {
                let ecu = factories
                    .create(&ecu_cfg.name, ecu_cfg.step_budget_us)
                    .map_err(|message| ModelError::Ecu {
                        ecu: ecu_cfg.name.clone(),
                        message,
                    })?;
                sim.attach(ecu, &ecu_cfg.listen)
            }
        };
        ecus.push((ecu_cfg.name.clone(), index));
    }

    Ok(BuiltSimulation { sim, ecus })
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_core::time::US_PER_MS;
    use std::io::Write;

    const DBC: &str = ecus::TESTS_DBC;

    fn write_temp(text: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("embrig-model-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("test.dbc");
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(text.as_bytes()).unwrap();
        path
    }

    fn config() -> VehicleConfig {
        let mut cfg = VehicleConfig::new("ev-powertrain");
        cfg.dbc = "test.dbc".into();
        cfg.step_us = US_PER_MS;
        cfg.ecus = vec![
            serde_saphyr::from_str(
                r#"
name: battery
type: config
message: BatteryStatus
period_us: 100000
signals:
  voltage: 400.0
  current: 0.0
  soc: 90.0
  state: READY
  contactor_closed: true
"#,
            )
            .unwrap(),
            serde_saphyr::from_str(
                r#"
name: brake
type: config
message: BrakeStatus
period_us: 100000
signals:
  brake_pressed: false
"#,
            )
            .unwrap(),
            serde_saphyr::from_str(
                r#"
name: driver
type: config
message: DriverRequest
period_us: 100000
signals:
  drive_enabled: true
"#,
            )
            .unwrap(),
            serde_saphyr::from_str(
                r#"
name: charger
type: charger
period_us: 100000
"#,
            )
            .unwrap(),
            serde_saphyr::from_str(
                r#"
name: vcu
type: vcu
period_us: 50000
"#,
            )
            .unwrap(),
            serde_saphyr::from_str(
                r#"
name: motor
type: motor
period_us: 50000
"#,
            )
            .unwrap(),
        ];
        cfg
    }

    #[test]
    fn builds_and_runs_powertrain() {
        let dbc = write_temp(DBC);
        let cfg = config();
        let mut sim = build_simulation(&cfg, &dbc).unwrap();
        sim.run_ms(200);
        let counts = sim.frame_counts();
        assert!(counts.iter().any(|(id, _)| *id == ecus::ID_BATTERY));
        assert!(counts.iter().any(|(id, _)| *id == ecus::ID_MOTOR_STATUS));
        let frame = sim.recorder().last_frame(ecus::ID_MOTOR_ENABLE).unwrap();
        let enable = embrig_dbc::parse(DBC)
            .unwrap()
            .message(ecus::ID_MOTOR_ENABLE)
            .unwrap()
            .decode_signal(&frame.data, "motor_enable")
            .unwrap();
        assert!(enable > 0.5);
    }

    #[test]
    fn yaml_roundtrip() {
        let cfg = config();
        let text = serde_saphyr::to_string(&cfg).unwrap();
        let parsed: VehicleConfig = serde_saphyr::from_str(&text).unwrap();
        assert_eq!(parsed.name, cfg.name);
        assert_eq!(parsed.ecus.len(), 6);
        assert_eq!(
            parsed.ecus[0].signals.get("state"),
            Some(&SignalLiteral::Str("READY".into()))
        );
    }
}
