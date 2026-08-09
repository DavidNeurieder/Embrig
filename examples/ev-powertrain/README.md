# EV Powertrain — OpenHIL example

A small electric-vehicle powertrain you can simulate on any laptop in under ten
minutes — and optionally run against one real ECU over CAN. This is the
reference "vertical" for OpenHIL.

## What it is

Four ECUs on one CAN bus:

| ECU | Role | Frames |
| --- | --- | --- |
| `battery` | static sensor node (config-driven) | `0x100` BatteryStatus |
| `brake` | static sensor node | `0x110` BrakeStatus |
| `driver` | static sensor node | `0x120` DriverRequest |
| `charger` | state machine (Rust) | `0x200`/`0x210` ChargeRequest/Status |
| `vcu` | **the ECU under test** (Rust) | `0x220` MotorEnable |
| `motor` | state machine (Rust) | `0x230` MotorStatus |

The VCU enables the motor only when **all** of these hold:

- battery reports `READY` with `voltage ≤ 450 V`,
- the brake is released and the driver requests drive,
- the charger is not in `FAULT`,
- a battery frame has been seen within the last 300 ms.

Any violation drops `motor_enable` to false and the motor falls back to `SAFE`.

## Run it

```sh
cargo run --release --bin openhil -- test examples/ev-powertrain/vehicle.yaml
```

or, installed:

```sh
openhil test examples/ev-powertrain/vehicle.yaml
```

Expected output: 5 passing tests. To see one fail, change `voltage: 460.0`
back to `400.0` in `tests/overvoltage.yaml` — the overvoltage test flips
PASS → FAIL because the safety check is gone.

Other commands:

```sh
openhil simulate examples/ev-powertrain/vehicle.yaml --duration 2s --verbose
openhil test examples/ev-powertrain/vehicle.yaml --report report.html
openhil init my-project          # scaffold your own vehicle
```

## How the tests work

Each `tests/*.yaml` file is a sequence of steps: `wait`, `send`, `set_signal`,
`fault`, `expect`. Assertions decode real signals through the DBC:

```yaml
name: overvoltage_disables_motor
timeout: 5s
steps:
  - wait: { time: 300ms }
  - set_signal: { ecu: battery, id: 0x100, signal: voltage, value: 460.0 }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
  - expect: { id: 0x230, signal: state, equals: "SAFE", within: 1s }
```

## Hardware mode (optional)

The same tests run against a real bus. Build with SocketCAN support:

```sh
cargo build --features socketcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
openhil test examples/ev-powertrain/vehicle.yaml --interface vcan0
```

`set_signal` and `fault` steps are virtual-only (there is no router on a real
bus) — they are rejected with a clear error instead of being silently ignored.

## Bill of materials

### Pure software (€0)

- Any laptop. That's it — the whole powertrain runs in the simulation.

### One real ECU (~€15–30)

Replace one virtual ECU with real hardware and let the bus run against it:

- 1× STM32 Nucleo board (e.g. Nucleo-F103RB, ~€12–20)
- 1× SN65HVD230 CAN transceiver module (~€3–8)
- jumper wires
- Firmware: run the `0x220` MotorEnable logic (or any ECU) on the STM32;
  OpenHIL's socketcan mode feeds it the same frames the virtual VCU received.

### Real CAN bus / full rig (~€15–50)

- 1× USB-CAN adapter (e.g. CANable, ~€15–30), *or*
- 1× Raspberry Pi + CAN HAT (~€30–50) using the Linux `can` / `vcan` stack
- 1× 120 Ω termination resistor on each bus end
- 2× SN65HVD230 transceiver modules if more than one real ECU

Everything except the USB-CAN adapter is just wires and cheap transceivers;
the socketcan interface on Linux is standard.

## Determinism note

The simulation steps in fixed integer-microsecond ticks; ECUs run in the order
listed in `vehicle.yaml`. Re-running a test yields identical results, so a
PASS→FAIL flip is always caused by the change you made — never by the runner.
