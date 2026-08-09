# OpenHIL MVP v3 — Build Plan

v2 was deliberately minimal (0→1 validation). We validated the wedge direction
and now commit to a **complete, usable v1 vision** — but kept small enough for
one developer to finish, and built around a **student-replicable vertical**.

> **User story:** a student or engineer clones a repo, runs `openhil test`, sees
> a virtual EV powertrain come alive on a laptop, watches a fault get injected,
> and gets a report that explains the failure. Optionally they swap in one real
> STM32 ECU over CAN and run the same tests against it.

---

## 1. Vertical: student EV powertrain (replicable)

Chosen because students can reproduce it with almost nothing:

| Mode | What a student needs | Cost |
| --- | --- | --- |
| Pure software | any laptop | €0 |
| One real ECU | STM32 Nucleo + SN65HVD230 CAN module | ~€15–30 |
| Real CAN bus | USB-CAN adapter (or Pi CAN HAT + SocketCAN) | ~€15–50 |

Domain (from `mvp_final.txt` Phase 0): **BMS + Charger + Vehicle Controller +
Motor** over CAN. Clear safety scenarios, CAN-heavy, easy to explain.

Scenarios students replicate:
- startup sequence (BMS ready → VCU enables motor)
- charger timeout → safe shutdown
- battery overvoltage → contactor open, motor disabled
- message loss → safe state entered
- regression: remove a safety check in firmware → test flips PASS→FAIL

---

## 2. Feature set (full v1 vision)

From `MVP_PLAN.md` v1, now committed:

- ✅ Rust simulation engine (event clock, deterministic stepping)
- ✅ Virtual CAN bus (routing, filtering, timing, fault injection)
- ✅ Virtual ECUs — config-driven + Rust models
- ✅ DBC parsing + signal encode/decode
- ✅ YAML test DSL (send / wait / expect / fault), assertions
- ✅ Reports — HTML + JSON (CI-friendly)
- ✅ SocketCAN hardware mode — same tests, one ECU real
- ✅ CLI: `openhil init / simulate / test / report`
- ✅ Student example project with README + BOM

Deferred (1→N, unchanged from v1): timeline/signal dashboards, SQLite/Parquet
storage, WASM plugin models, model registry, UDS/ISO-TP/J1939/LIN/CAN FD.

---

## 3. Architecture

Cargo workspace:

```text
openhil/
├── Cargo.toml                 workspace
├── crates/
│   ├── openhil-core/          CanFrame, Timestamp, Bus, Ecu trait, Simulation, Recorder, faults
│   ├── openhil-dbc/           DBC parser + signal encoding (Intel/Motorola, signed, factor/offset)
│   ├── openhil-test/          YAML DSL, step runner, assertions, report (HTML/JSON)
│   ├── openhil-can/           CanInterface trait + virtual + SocketCAN backends
│   └── openhil-cli/           `openhil` binary
└── examples/ev-powertrain/    vehicle.yaml, powertrain.dbc, tests/, README.md
```

Core abstraction (unchanged from v1/v2):

```rust
trait Ecu: Send {
    fn update(&mut self, time: Timestamp);
    fn on_message(&mut self, frame: &CanFrame);
    fn transmit(&mut self, time: Timestamp, out: &mut Vec<CanFrame>);
}
```

A `Bus` routes frames between ECUs and external targets. The **same bus** is used
for virtual simulation and hardware mode: in hardware mode one ECU is replaced by
a SocketCAN channel; frames flow in/out unchanged. Tests therefore run identically
in both modes (the HIL progression from `vEcu.txt`).

---

## 4. Reference vECUs

1. **Battery (config-driven + Rust behavior)** — `0x100` BatteryStatus (voltage,
   current, SOC, state). Behaviors: startup, charging, discharging, fault.
2. **Charger (Rust)** — ChargeRequest/ChargeStatus; accept, reject unsafe, timeout.
3. **Vehicle Controller (Rust)** — BatteryStatus + BrakeStatus + DriverRequest →
   MotorEnable. The "logic under test".
4. **Motor (config-driven)** — MotorStatus, enable handling.

---

## 5. Test DSL

```yaml
name: charger_timeout_safe_state
timeout: 5s

steps:
  - send: { id: 0x200, data: [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] }
  - wait: { time: 500ms }
  - expect: { id: 0x210, signal: state, equals: "CHARGING" }
  - fault: { type: drop, id: 0x100, duration: 1000ms }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
```

Assertions: `equals`, `greater_than`, `less_than`, `present`, `absent`, `timeout`.
Signal-based steps resolve against the loaded DBC.

---

## 6. Build phases

| Phase | Scope | Week |
| --- | --- | --- |
| 1 | core crate: frames, bus, clock, simulation, recorder, faults | 1 |
| 2 | dbc crate: parser + signal codec + tests | 1–2 |
| 3 | vECUs: config-driven + charger/VCU/motor models | 2–3 |
| 4 | test crate: DSL, runner, assertions, HTML/JSON reports | 3–4 |
| 5 | CLI + SocketCAN hardware mode | 4–5 |
| 6 | example project + student README, end-to-end verification | 5 |

## 7. Success criteria

- `examples/ev-powertrain`: `openhil test tests/` → PASS/FAIL with a readable
  report explaining each failure.
- Same tests run with `--target can0` against one real STM32 ECU.
- A student can replicate the pure-software demo on any laptop in < 10 minutes,
  and the hardware demo for < €50.
- Repo layout is clean enough to recruit contributors (per `mvp.txt`).
