# Embrig Implementation Plan

Goal: build the full v1 vision (per `MVP_PLAN_V3.md`) — Rust workspace with a
deterministic virtual CAN simulation, DBC support, YAML test runner, SocketCAN
hardware mode, software-in-the-loop (host-compiled firmware on the virtual
bus), an Ethernet (UDP) transport (virtual + SIL + hardware), and a
student-replicable EV powertrain example.

## Locked decisions

- **YAML**: `serde-saphyr 1.0.1` (serde_yaml is deprecated)
- **Async**: tokio for the runner + hardware I/O; core simulation stays
  synchronous and deterministic (virtual tests never use wall clock)
- **SocketCAN**: feature-gated (`socketcan` feature), uses `socketcan 3.6.2`
  with its native `tokio` integration
- **Ethernet (UDP)**: transport-only `embrig-net` crate (netmap codec +
  `UdpSim`), targets live in `embrig-test::udp`; `embrig-net` depends only on
  `embrig-core`, never on `embrig-test` (avoids a dependency cycle)
- **Time**: integer microseconds (`u64`); ECUs stepped in config order (no
  HashMap iteration) → reproducible results

## Workspace layout

```text
crates/
  embrig-core     std-only types: CanFrame, Timestamp, Ecu trait, Fault,
                   Simulation, Recorder. NO async.
  embrig-dbc      DBC parser (BO_/SG_/VAL_) + signal codec
                   (Intel/Motorola, signed, factor/offset). thiserror.
  embrig-models   VehicleConfig (YAML), ConfigEcu, Rust vECUs
                   (Charger, VehicleController, Motor), Simulation builder.
  embrig-net      Deterministic virtual UDP network: netmap codec, UdpDatagram,
                   UdpEcu trait, UdpConfigEcu, UdpRegistry, UdpSim with routing
                   + drop/delay/corrupt faults. NO tokio/test deps.
  embrig-can      SocketCanBus async backend (feature socketcan). thiserror.
  embrig-sil      Software-in-the-loop: SilRegistry, SilTarget, sil_run,
                   wall-clock step budgets. Host-compiled firmware on the
                   virtual bus.
  embrig-test     YAML test DSL, async runner (virtual + SIL + UDP + hardware
                   targets), assertions, HTML/JSON reports. serde + saphyr + tokio + serde_json.
  embrig-cli      binary `embrig`: init / simulate / test / report.
                   clap + anyhow. #[tokio::main].
examples/ev-powertrain  vehicle.yaml, powertrain.dbc, tests/*.yaml, README.md (student BOM)
examples/sil (crates/embrig-sil/examples)  thermal-controller (sil_firmware.rs)
  + diff-drive rover (robot_sil.rs): fixtures, suites, firmware
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

## SIL (software-in-the-loop)

Firmware = code, not config. A `type: sil` ECU node in `vehicle.yaml` is bound
at runtime to host-compiled firmware via an `EcuFactory`:

```yaml
ecus:
  - name: controller
    type: sil
    period_us: 50000
    listen: [0x100]
    step_budget_us: 100000   # default 100 ms per simulated step
```

```rust
let mut registry = SilRegistry::new();
registry.register("controller", |name, _| Ok(Box::new(ControllerFirmware::new(name))));
let result = sil_run(&config, &dbc, registry, &suites)?;
```

- `SilRegistry` maps node name → factory; unknown `sil` nodes fail at startup
  with a clear "no firmware registered" error listing what is registered.
- `SilTarget` mirrors `VirtualTarget` as a `TestTarget`: same YAML suites, same
  fault injection. `set_signal` on the firmware itself is rejected
  (`UnsupportedOnSut`) — the DUT is driven through the bus.
- Each firmware step runs under a wall-clock budget; overruns fail the test
  (`SutTimeout`) instead of hanging it.
- `reset()` rebuilds the simulation and re-runs the factories, so firmware
  state never leaks between tests.
- The CLI does not run `--interface sil`; SIL is used from the `embrig-sil`
  crate (bundled `sil_firmware` example: 2/2 suites pass).

## UDP (Ethernet) transport

Ethernet nodes are described by a **netmap** instead of a DBC: each message is
keyed by its destination endpoint and carries named fields at byte offsets
(`u8`, `bool`, `u16le`/`u16be`, `u32le`/`u32be`, `i16le`, `i32le`, `f32le`,
`f64le`, with scaling and symbolic values). In `vehicle.yaml`, `eth_ecus:`
(`type: udp-config` = config-driven, `type: udp-sil` = host-compiled firmware)
and `networks:` (host endpoint + netmap path) replace/augment `ecus:` + `dbc:`;

```yaml
eth_ecus:
  - { name: motion, type: udp-config, address: 192.168.1.30:5000,
      message: MotionState, period_us: 50000, fields: { speed: 0.0, state: STOPPED } }
networks:
  - { name: eth, type: udp, host: 192.168.1.10:5000, netmap: netmap.yaml }
interfaces:
  - { name: udp, type: udp }
```

The same DSL adds four steps — `send_udp`, `set_field`, `expect_udp`,
`fault_udp` (drop/delay/corrupt by message name) — and the targets mirror the
CAN ones: `UdpTarget` (virtual), `UdpSutTarget` (firmware, `catch_unwind` →
`SutTimeout`), `UdpHardwareTarget` (bound UDP socket). A vehicle with no `dbc:`
is pure-Ethernet; `--interface udp` selects the target and DBC loading is
skipped. Bundled demo: `crates/embrig-test/examples/rover/` (`udp_rover`
example: 4/4 suites pass).

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

Step kinds: `send` (raw), `set_signal`, `wait`, `expect`, `fault` — plus the
UDP equivalents `send_udp`, `set_field`, `expect_udp`, `fault_udp`.
Assertions: `equals`, `greater_than`, `less_than`, `present`, `absent`,
`within`. Durations support `us/ms/s`. Hardware target: `drop`/`set_signal`
unsupported (no CAN router in path) — documented + error.

## Exit codes

`0` all pass · `1` test failures · `2` config/load/usage errors.

## Implementation order + verification

1. workspace + `embrig-core` → unit tests (bus/faults/clock)
2. `embrig-dbc` → codec tests incl. round-trip + known vectors
3. `embrig-models` → vECU behavior tests
4. `embrig-can` → compiles feature-gated
5. `embrig-net` → netmap codec, ecu, sim tests (routing/faults/determinism)
6. `embrig-test` → DSL/assertion tests, runner tests, `udp` targets + suites
7. `embrig-sil` → registry/budget/target tests, `sil_firmware` example
8. `embrig-cli` → black-box tests (exit codes, report files)
9. `examples/ev-powertrain` → demo scenarios run green + regression proof
10. CI workflow; `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`,
    `cargo test`, optional vcan0 job
