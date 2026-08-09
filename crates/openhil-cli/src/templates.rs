//! Static file templates for `openhil init`.

/// The powertrain DBC matching the message map in `IMPLEMENTATION_PLAN.md`.
pub const POWERTRAIN_DBC: &str = r#"VERSION "0.1"

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

pub const VEHICLE_YAML: &str = r#"# OpenHIL vehicle definition.
# ECUs are stepped in this order, so the simulation is deterministic.
name: ev-powertrain
dbc: powertrain.dbc
step_us: 1000

ecus:
  # Static nodes: the test runner overrides their signals to inject stimulus.
  - name: battery
    type: config
    message: BatteryStatus
    period_us: 100000
    signals:
      voltage: 400.0
      current: 0.0
      soc: 90.0
      state: "READY"
      contactor_closed: true

  - name: brake
    type: config
    message: BrakeStatus
    period_us: 100000
    signals:
      brake_pressed: false

  - name: driver
    type: config
    message: DriverRequest
    period_us: 100000
    signals:
      drive_enabled: true

  - name: charger
    type: charger
    period_us: 100000
    listen: [0x200, 0x100]

  # The ECU under test.
  - name: vcu
    type: vcu
    period_us: 50000
    listen: [0x100, 0x110, 0x120, 0x210]

  - name: motor
    type: motor
    period_us: 50000
    listen: [0x220]

interfaces:
  - name: virtual
    type: virtual
  - name: vcan0
    type: socketcan
    interface: vcan0
"#;

pub const TEST_NOMINAL: &str = r#"# Under nominal conditions the VCU enables the motor and it runs.
name: nominal_conditions_enable_motor
timeout: 5s
steps:
  - wait: { time: 300ms }
  - expect: { id: 0x220, signal: motor_enable, equals: true, within: 1s }
  - expect: { id: 0x230, signal: state, equals: "RUNNING", within: 1s }
"#;

pub const TEST_OVERVOLTAGE: &str = r#"# Over 450 V the VCU refuses to enable the motor (safe state).
name: overvoltage_disables_motor
timeout: 5s
steps:
  - wait: { time: 300ms }
  - set_signal: { ecu: battery, id: 0x100, signal: voltage, value: 460.0 }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
  - expect: { id: 0x230, signal: state, equals: "SAFE", within: 1s }
"#;

pub const TEST_BRAKE: &str = r#"# Pressing the brake immediately releases the motor.
name: brake_press_disables_motor
timeout: 5s
steps:
  - wait: { time: 300ms }
  - set_signal: { ecu: brake, id: 0x110, signal: brake_pressed, value: true }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
"#;

pub const TEST_CHARGER_FAULT: &str = r#"# Request charging, then drop the battery bus: the charger must fault.
name: charger_faults_on_stale_battery
timeout: 5s
steps:
  - send: { id: 0x200, data: [1, 0, 0, 0, 0, 0, 0, 0] }
  - wait: { time: 300ms }
  - expect: { id: 0x210, signal: state, equals: "CHARGING", within: 1s }
  - fault: { type: drop, id: 0x100, duration: 1000ms }
  - expect: { id: 0x210, signal: state, equals: "FAULT", within: 2s }
"#;

pub const TEST_PRESENT: &str = r#"# The bus carries periodic battery and brake frames.
name: bus_carries_periodic_frames
timeout: 5s
steps:
  - wait: { time: 200ms }
  - expect: { id: 0x100, present: true, within: 500ms }
  - expect: { id: 0x110, present: true, within: 500ms }
  - expect: { id: 0x230, present: true, within: 500ms }
"#;
