# OpenHIL Implementation Plan

Goal: build the full v1 vision (per `MVP_PLAN_V3.md`) — Rust workspace with a
deterministic virtual CAN simulation, DBC support, YAML test runner, SocketCAN
hardware mode, and a student-replicable EV powertrain example.

## Locked decisions

- **YAML**: `serde-saphyr 1.0.1` (serde_yaml is deprecated)
- **Async**: tokio for the runner + hardware I/O; core simulation stays
  synchronous and deterministic (virtual tests never use wall clock)
- **SocketCAN**: feature-gated (`socketcan` feature), uses `socketcan 3.6.2`
  with its native `tokio` integration
- **Time**: integer microseconds (`u64`); ECUs stepped in config order (no
  HashMap iteration) → reproducible results

## Workspace layout

```text
crates/
  openhil-core     std-only types: CanFrame, Timestamp, Ecu trait, Fault,
                   Simulation, Recorder. NO async.
  openhil-dbc      DBC parser (BO_/SG_/VAL_) + signal codec
                   (Intel/Motorola, signed, factor/offset). thiserror.
  openhil-models   VehicleConfig (YAML), ConfigEcu, Rust vECUs
                   (Charger, VehicleController, Motor), Simulation builder.
  openhil-can      SocketCanBus async backend (feature socketcan). thiserror.
  openhil-test     YAML test DSL, async runner (virtual + hardware targets),
                   assertions, HTML/JSON reports. serde + saphyr + tokio + serde_json.
  openhil-cli      binary `openhil`: init / simulate / test / report.
                   clap + anyhow. #[tokio::main].
examples/ev-powertrain  vehicle.yaml, powertrain.dbc, tests/*.yaml, README.md (student BOM)
.github/workflows/ci.yml
```

## Message map (ev-powertrain, powertrain.dbc)

| ID | Message | Signals |
| --- | --- | --- |
| 0x100 | BatteryStatus | voltage(0.1V), current(0.1A), soc(%), state, contactor_closed |
| 0x110 | BrakeStatus | brake_pressed |
| 0x120 | DriverRequest | drive_enabled |
| 0x200 | ChargeRequest | charge_request |
| 0x210 | ChargeStatus | state |
| 0x220 | MotorEnable | motor_enable |
| 0x230 | MotorStatus | state, rpm |

State value tables (VAL_): battery OFF/INIT/READY/CHARGING/FAULT,
charger IDLE/CHARGING/COMPLETE/FAULT, motor OFF/READY/RUNNING/SAFE/FAULT.

## vECU behavior

- **Battery** (config-driven): static READY values; test overrides voltage via
  `set_signal` to inject overvoltage. Periodic 0x100 @ 100 ms.
- **Charger** (Rust): IDLE → CHARGING on request if battery healthy; FAULT if
  0x100 missing >500 ms. Periodic 0x210 @ 100 ms.
- **VehicleController** (Rust, ECU under test): motor_enable = battery READY &&
  voltage ≤ 450 && brake released && drive requested && charger ok &&
  battery msg not timed out (>300 ms). Periodic 0x220 @ 50 ms.
- **Motor** (Rust): RUNNING on enable, else SAFE. Periodic 0x230 @ 50 ms.
- **Brake / Driver** (config-driven): static; overridden by tests.

## Test DSL

```yaml
name: charger_timeout_safe_state
timeout: 5s
steps:
  - send: { id: 0x200, data: [0x01,0,0,0,0,0,0,0] }
  - wait: { time: 500ms }
  - expect: { id: 0x210, signal: state, equals: "CHARGING" }
  - fault: { type: drop, id: 0x100, duration: 1000ms }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
```

Step kinds: `send` (raw), `set_signal`, `wait`, `expect`, `fault`.
Assertions: `equals`, `greater_than`, `less_than`, `present`, `absent`,
`within`. Durations support `us/ms/s`. Hardware target: `drop`/`set_signal`
unsupported (no CAN router in path) — documented + error.

## Exit codes

`0` all pass · `1` test failures · `2` config/load/usage errors.

## Implementation order + verification

1. workspace + `openhil-core` → unit tests (bus/faults/clock)
2. `openhil-dbc` → codec tests incl. round-trip + known vectors
3. `openhil-models` → vECU behavior tests
4. `openhil-can` → compiles feature-gated
5. `openhil-test` → DSL/assertion tests, runner tests
6. `openhil-cli` → black-box tests (exit codes, report files)
7. `examples/ev-powertrain` → demo scenarios run green + regression proof
8. CI workflow; `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`,
   `cargo test`, optional vcan0 job
