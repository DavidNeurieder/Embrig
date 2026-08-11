//! Reference vECUs for the EV powertrain.
//!
//! These implement the message map fixed in the implementation plan:
//!
//! | ID   | Message        | Direction                       |
//! |------|----------------|----------------------------------|
//! | 0x100 | BatteryStatus | battery → bus                   |
//! | 0x110 | BrakeStatus   | brake → bus                     |
//! | 0x120 | DriverRequest | driver → bus                    |
//! | 0x200 | ChargeRequest | charger ← bus                   |
//! | 0x210 | ChargeStatus  | charger → bus                   |
//! | 0x220 | MotorEnable   | VCU → bus                       |
//! | 0x230 | MotorStatus   | motor → bus                     |
//!
//! The VCU is the ECU under test: its `motor_enable` output encodes the
//! safety decision derived from its inputs.

use embrig_core::codec::MessageCodec;
use embrig_core::frame::CanFrame;
use embrig_core::time::{ms, Timestamp};
use embrig_core::{EcuError, NetEcu};

/// Message ids from the powertrain DBC.
pub const ID_BATTERY: u32 = 0x100;
pub const ID_BRAKE: u32 = 0x110;
pub const ID_DRIVER: u32 = 0x120;
pub const ID_CHARGE_REQUEST: u32 = 0x200;
pub const ID_CHARGE_STATUS: u32 = 0x210;
pub const ID_MOTOR_ENABLE: u32 = 0x220;
pub const ID_MOTOR_STATUS: u32 = 0x230;

/// Signal-value-table symbols.
pub const BATTERY_READY: &str = "READY";
pub const BATTERY_FAULT: &str = "FAULT";
pub const CHARGER_IDLE: &str = "IDLE";
pub const CHARGER_CHARGING: &str = "CHARGING";
pub const CHARGER_COMPLETE: &str = "COMPLETE";
pub const CHARGER_FAULT: &str = "FAULT";
pub const MOTOR_RUNNING: &str = "RUNNING";
pub const MOTOR_SAFE: &str = "SAFE";

/// Resolution of a value-table symbol to a physical value for `name`.
fn symbol_physical(message: &dyn MessageCodec, name: &str, symbol: &str) -> Option<f64> {
    message.physical_for_symbol(name, symbol)
}

/// The charger: `IDLE → CHARGING` on request while the battery is healthy,
/// `FAULT` when the battery message is stale for more than 500 ms.
pub struct Charger {
    name: String,
    request: Box<dyn MessageCodec>,
    battery: Box<dyn MessageCodec>,
    status: Box<dyn MessageCodec>,
    period: Timestamp,
    next: Timestamp,
    state: &'static str,
    charge_request: bool,
    last_battery: Option<Timestamp>,
    last_soc: Option<f64>,
}

impl Charger {
    pub fn new(
        name: impl Into<String>,
        request: Box<dyn MessageCodec>,
        battery: Box<dyn MessageCodec>,
        status: Box<dyn MessageCodec>,
        period: Timestamp,
    ) -> Self {
        Self {
            name: name.into(),
            request,
            battery,
            status,
            period,
            next: 0,
            state: CHARGER_IDLE,
            charge_request: false,
            last_battery: None,
            last_soc: None,
        }
    }

    fn battery_fresh(&self, time: Timestamp) -> bool {
        match self.last_battery {
            Some(t) => time.saturating_sub(t) <= ms(500),
            None => false,
        }
    }
}

impl NetEcu<CanFrame> for Charger {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, time: Timestamp) {
        if frame.id == self.request.id() {
            if let Ok(v) = self.request.decode_signal(&frame.data, "charge_request") {
                self.charge_request = v > 0.5;
            }
        }
        if frame.id == self.battery.id() {
            self.last_battery = Some(time);
            if let Ok(soc) = self.battery.decode_signal(&frame.data, "soc") {
                self.last_soc = Some(soc);
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if self.charge_request {
            if !self.battery_fresh(time) {
                self.state = CHARGER_FAULT;
            } else if self.last_soc.map(|s| s >= 100.0).unwrap_or(false) {
                self.state = CHARGER_COMPLETE;
            } else {
                self.state = CHARGER_CHARGING;
            }
        } else {
            self.state = CHARGER_IDLE;
        }

        if time < self.next {
            return;
        }
        if let Some(value) = symbol_physical(&*self.status, "state", self.state) {
            if let Ok(data) = self.status.encode_signals(&[("state", value)]) {
                if let Ok(frame) = CanFrame::new(self.status.id(), data) {
                    out.push(frame);
                }
            }
        }
        self.next = time + self.period;
    }
}

/// The VCU (ECU under test).
///
/// `motor_enable` is true only when:
/// - the battery reported `READY`, with `voltage ≤ 450 V`,
/// - the brake is released and the driver requests drive,
/// - the charger is not in `FAULT`,
/// - a battery message has been seen within the last 300 ms.
pub struct VehicleController {
    name: String,
    battery: Box<dyn MessageCodec>,
    brake: Box<dyn MessageCodec>,
    driver: Box<dyn MessageCodec>,
    charger: Box<dyn MessageCodec>,
    enable: Box<dyn MessageCodec>,
    period: Timestamp,
    next: Timestamp,
    battery_state: Option<&'static str>,
    battery_voltage: Option<f64>,
    last_battery: Option<Timestamp>,
    brake_pressed: Option<bool>,
    drive_enabled: Option<bool>,
    charger_state: Option<&'static str>,
}

impl VehicleController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: impl Into<String>,
        battery: Box<dyn MessageCodec>,
        brake: Box<dyn MessageCodec>,
        driver: Box<dyn MessageCodec>,
        charger: Box<dyn MessageCodec>,
        enable: Box<dyn MessageCodec>,
        period: Timestamp,
    ) -> Self {
        Self {
            name: name.into(),
            battery,
            brake,
            driver,
            charger,
            enable,
            period,
            next: 0,
            battery_state: None,
            battery_voltage: None,
            last_battery: None,
            brake_pressed: None,
            drive_enabled: None,
            charger_state: None,
        }
    }

    fn charger_ok(&self) -> bool {
        self.charger_state != Some(CHARGER_FAULT)
    }

    fn battery_fresh(&self, time: Timestamp) -> bool {
        match self.last_battery {
            Some(t) => time.saturating_sub(t) <= ms(300),
            None => false,
        }
    }
}

impl NetEcu<CanFrame> for VehicleController {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, time: Timestamp) {
        if frame.id == self.battery.id() {
            self.last_battery = Some(time);
            if let Ok(v) = self.battery.decode_signal(&frame.data, "voltage") {
                self.battery_voltage = Some(v);
            }
            if let Ok(v) = self.battery.decode_signal(&frame.data, "state") {
                let raw = v.round() as i64;
                self.battery_state =
                    self.battery
                        .symbol_for("state", raw)
                        .and_then(|s| match s.as_str() {
                            "OFF" => Some("OFF"),
                            "INIT" => Some("INIT"),
                            "READY" => Some("READY"),
                            "CHARGING" => Some("CHARGING"),
                            "FAULT" => Some("FAULT"),
                            _ => None,
                        });
            }
        } else if frame.id == self.brake.id() {
            if let Ok(v) = self.brake.decode_signal(&frame.data, "brake_pressed") {
                self.brake_pressed = Some(v > 0.5);
            }
        } else if frame.id == self.driver.id() {
            if let Ok(v) = self.driver.decode_signal(&frame.data, "drive_enabled") {
                self.drive_enabled = Some(v > 0.5);
            }
        } else if frame.id == self.charger.id() {
            if let Ok(v) = self.charger.decode_signal(&frame.data, "state") {
                let raw = v.round() as i64;
                self.charger_state =
                    self.charger
                        .symbol_for("state", raw)
                        .and_then(|s| match s.as_str() {
                            "IDLE" => Some("IDLE"),
                            "CHARGING" => Some("CHARGING"),
                            "COMPLETE" => Some("COMPLETE"),
                            "FAULT" => Some("FAULT"),
                            _ => None,
                        });
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next {
            return;
        }
        let battery_ok = self.battery_state == Some(BATTERY_READY)
            && self.battery_voltage.map(|v| v <= 450.0).unwrap_or(false)
            && self.battery_fresh(time);
        let driver_ok = self.drive_enabled == Some(true) && self.brake_pressed == Some(false);
        let enable = battery_ok && driver_ok && self.charger_ok();
        if let Ok(data) = self
            .enable
            .encode_signals(&[("motor_enable", if enable { 1.0 } else { 0.0 })])
        {
            if let Ok(frame) = CanFrame::new(self.enable.id(), data) {
                out.push(frame);
            }
        }
        self.next = time + self.period;
    }

    fn set_signal(
        &mut self,
        _id: u32,
        _signal: &str,
        _value: embrig_core::signal::SignalValue,
    ) -> Result<(), EcuError> {
        Err(EcuError::SignalNotSupported)
    }
}

/// The motor: `RUNNING` when enabled, otherwise `SAFE`.
pub struct Motor {
    name: String,
    enable: Box<dyn MessageCodec>,
    status: Box<dyn MessageCodec>,
    period: Timestamp,
    next: Timestamp,
    enabled: bool,
}

impl Motor {
    pub fn new(
        name: impl Into<String>,
        enable: Box<dyn MessageCodec>,
        status: Box<dyn MessageCodec>,
        period: Timestamp,
    ) -> Self {
        Self {
            name: name.into(),
            enable,
            status,
            period,
            next: 0,
            enabled: false,
        }
    }
}

impl NetEcu<CanFrame> for Motor {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        if frame.id == self.enable.id() {
            if let Ok(v) = self.enable.decode_signal(&frame.data, "motor_enable") {
                self.enabled = v > 0.5;
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next {
            return;
        }
        let state = if self.enabled {
            MOTOR_RUNNING
        } else {
            MOTOR_SAFE
        };
        let rpm = if self.enabled { 3000.0 } else { 0.0 };
        if let Some(state_v) = symbol_physical(&*self.status, "state", state) {
            if let Ok(data) = self
                .status
                .encode_signals(&[("state", state_v), ("rpm", rpm)])
            {
                if let Ok(frame) = CanFrame::new(self.status.id(), data) {
                    out.push(frame);
                }
            }
        }
        self.next = time + self.period;
    }
}

/// DBC fixture shared by the models tests (see also `tests_dbc`).
#[cfg(test)]
pub(crate) const TESTS_DBC: &str = r#"
VERSION "0.1"

NS_ :

BS_:

BU_: Vector__XXX

BO_ 256 BatteryStatus: 8 Vector__XXX
 SG_ voltage : 0|16@1+ (0.1,0) [0|600] "V"  Vector__XXX
 SG_ current : 16|16@1+ (0.1,0) [-500|500] "A"  Vector__XXX
 SG_ soc : 32|8@1+ (1,0) [0|100] "%"  Vector__XXX
 SG_ state : 40|8@1+ (1,0) [0|4] ""  Vector__XXX
 SG_ contactor_closed : 48|1@1+ (1,0) [0|1] ""  Vector__XXX

BO_ 272 BrakeStatus: 8 Vector__XXX
 SG_ brake_pressed : 0|1@1+ (1,0) [0|1] ""  Vector__XXX

BO_ 288 DriverRequest: 8 Vector__XXX
 SG_ drive_enabled : 0|1@1+ (1,0) [0|1] ""  Vector__XXX

BO_ 512 ChargeRequest: 8 Vector__XXX
 SG_ charge_request : 0|1@1+ (1,0) [0|1] ""  Vector__XXX

BO_ 528 ChargeStatus: 8 Vector__XXX
 SG_ state : 0|8@1+ (1,0) [0|3] ""  Vector__XXX

BO_ 544 MotorEnable: 8 Vector__XXX
 SG_ motor_enable : 0|1@1+ (1,0) [0|1] ""  Vector__XXX

BO_ 560 MotorStatus: 8 Vector__XXX
 SG_ state : 0|8@1+ (1,0) [0|4] ""  Vector__XXX
 SG_ rpm : 8|16@1+ (1,0) [0|12000] "rpm"  Vector__XXX

VAL_ 256 state 0 "OFF" 1 "INIT" 2 "READY" 3 "CHARGING" 4 "FAULT" ;
VAL_ 528 state 0 "IDLE" 1 "CHARGING" 2 "COMPLETE" 3 "FAULT" ;
VAL_ 560 state 0 "OFF" 1 "READY" 2 "RUNNING" 3 "SAFE" 4 "FAULT" ;
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ConfigEcu, SignalLiteral};
    use embrig_core::simulation::Simulation;
    use embrig_core::time::US_PER_MS;
    use embrig_core::SignalValue;
    use std::collections::BTreeMap;

    fn sigs(map: &[(&str, SignalLiteral)]) -> BTreeMap<String, SignalLiteral> {
        map.iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    /// Assemble the full powertrain on a virtual bus.
    fn powertrain() -> (Simulation, usize, usize, usize, usize) {
        let network = embrig_dbc::parse(TESTS_DBC).unwrap();
        let msg = |id: u32| network.message(id).unwrap().clone();

        let mut sim = Simulation::new(US_PER_MS);

        let battery = ConfigEcu::new(
            "battery",
            Box::new(msg(ID_BATTERY)),
            ms(100),
            &sigs(&[
                ("voltage", SignalLiteral::Num(400.0)),
                ("current", SignalLiteral::Num(0.0)),
                ("soc", SignalLiteral::Num(90.0)),
                ("state", SignalLiteral::Str(BATTERY_READY.into())),
                ("contactor_closed", SignalLiteral::Bool(true)),
            ]),
        )
        .unwrap();
        let brake = ConfigEcu::new(
            "brake",
            Box::new(msg(ID_BRAKE)),
            ms(100),
            &sigs(&[("brake_pressed", SignalLiteral::Bool(false))]),
        )
        .unwrap();
        let driver = ConfigEcu::new(
            "driver",
            Box::new(msg(ID_DRIVER)),
            ms(100),
            &sigs(&[("drive_enabled", SignalLiteral::Bool(true))]),
        )
        .unwrap();

        let charger = Charger::new(
            "charger",
            Box::new(msg(ID_CHARGE_REQUEST)),
            Box::new(msg(ID_BATTERY)),
            Box::new(msg(ID_CHARGE_STATUS)),
            ms(100),
        );
        let vcu = VehicleController::new(
            "vcu",
            Box::new(msg(ID_BATTERY)),
            Box::new(msg(ID_BRAKE)),
            Box::new(msg(ID_DRIVER)),
            Box::new(msg(ID_CHARGE_STATUS)),
            Box::new(msg(ID_MOTOR_ENABLE)),
            ms(50),
        );
        let motor = Motor::new(
            "motor",
            Box::new(msg(ID_MOTOR_ENABLE)),
            Box::new(msg(ID_MOTOR_STATUS)),
            ms(50),
        );

        let battery_i = sim.attach(Box::new(battery), &[]);
        let brake_i = sim.attach(Box::new(brake), &[]);
        let driver_i = sim.attach(Box::new(driver), &[]);
        let _charger_i = sim.attach(Box::new(charger), &[ID_CHARGE_REQUEST, ID_BATTERY]);
        let _vcu_i = sim.attach(
            Box::new(vcu),
            &[ID_BATTERY, ID_BRAKE, ID_DRIVER, ID_CHARGE_STATUS],
        );
        let _motor_i = sim.attach(Box::new(motor), &[ID_MOTOR_ENABLE]);
        (sim, battery_i, brake_i, driver_i, _charger_i)
    }

    fn last_symbol(sim: &Simulation, id: u32, signal: &str) -> Option<String> {
        let frame = sim.recorder().last_message(&id)?;
        let network = embrig_dbc::parse(TESTS_DBC).unwrap();
        let m = network.message(id)?;
        let raw = m.decode_signal(&frame.data, signal).ok()?.round() as i64;
        m.symbol_for(signal, raw)
    }

    fn motor_enable(sim: &Simulation) -> Option<bool> {
        let frame = sim.recorder().last_message(&ID_MOTOR_ENABLE)?;
        Some(
            embrig_dbc::parse(TESTS_DBC)
                .unwrap()
                .message(ID_MOTOR_ENABLE)?
                .decode_signal(&frame.data, "motor_enable")
                .ok()?
                > 0.5,
        )
    }

    #[test]
    fn nominal_conditions_enable_motor() {
        let (mut sim, _, _, _, _) = powertrain();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(true));
        assert_eq!(
            last_symbol(&sim, ID_MOTOR_STATUS, "state").as_deref(),
            Some(MOTOR_RUNNING)
        );
    }

    #[test]
    fn overvoltage_disables_motor() {
        let (mut sim, battery_i, _, _, _) = powertrain();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(true));
        sim.set_signal(battery_i, ID_BATTERY, "voltage", SignalValue::Num(500.0))
            .unwrap();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(false));
        assert_eq!(
            last_symbol(&sim, ID_MOTOR_STATUS, "state").as_deref(),
            Some(MOTOR_SAFE)
        );
    }

    #[test]
    fn brake_press_disables_motor() {
        let (mut sim, _, brake_i, _, _) = powertrain();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(true));
        sim.set_signal(brake_i, ID_BRAKE, "brake_pressed", SignalValue::Num(1.0))
            .unwrap();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(false));
    }

    #[test]
    fn lost_battery_message_times_out_motor() {
        let (mut sim, _, _, _, _) = powertrain();
        sim.run_ms(200);
        assert_eq!(motor_enable(&sim), Some(true));
        // Stop the battery transmitting: its last frame was at t=200.
        sim.add_fault(embrig_core::fault::FaultRule {
            fault: embrig_core::fault::Fault::DropFrame { id: ID_BATTERY },
            start: ms(200),
            duration: None,
        });
        sim.run_ms(1000);
        // Motor is disabled once the battery has been silent > 300 ms.
        assert_eq!(motor_enable(&sim), Some(false));
    }

    #[test]
    fn charger_faults_when_battery_stale() {
        let (mut sim, _, _, _, _) = powertrain();
        sim.run_ms(100);
        assert_eq!(
            last_symbol(&sim, ID_CHARGE_STATUS, "state").as_deref(),
            Some(CHARGER_IDLE)
        );
        // Request charging via a raw frame.
        let req =
            embrig_core::frame::CanFrame::new(ID_CHARGE_REQUEST, vec![1, 0, 0, 0, 0, 0, 0, 0])
                .unwrap();
        sim.inject(req);
        sim.run_ms(100);
        assert_eq!(
            last_symbol(&sim, ID_CHARGE_STATUS, "state").as_deref(),
            Some(CHARGER_CHARGING)
        );
        // Drop the battery: charger goes FAULT after 500 ms.
        sim.add_fault(embrig_core::fault::FaultRule {
            fault: embrig_core::fault::Fault::DropFrame { id: ID_BATTERY },
            start: ms(200),
            duration: None,
        });
        sim.run_ms(1000);
        assert_eq!(
            last_symbol(&sim, ID_CHARGE_STATUS, "state").as_deref(),
            Some(CHARGER_FAULT)
        );
    }
}
