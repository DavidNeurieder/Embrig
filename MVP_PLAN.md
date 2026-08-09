# OpenHIL MVP Plan

Synthesized from the ideas in `ideas/`. The MVP's one job is to prove:

> **A developer can define a virtual embedded system, connect a real or simulated ECU, run automated tests, and get a useful report — without a €100k HIL rack.**

This is not "free CANoe." It is the missing middle between *manual CAN scripts* and *professional HIL systems*, built for how software engineers actually work today.

---

## 1. What we are NOT building

Explicitly out of scope for the MVP (per `mvp.txt`, `mvp2.txt`, `mvp_final.txt`):

- GUI model editor, GUI-heavy config editors
- Custom hardware / PCBs (support existing interfaces only)
- Full vehicle physics / 3D vehicle simulation
- AUTOSAR, FlexRay, LIN, J1939, SOME/IP
- Full UDS diagnostic stack, flashing tools, CAPL replacement
- AI-assisted debugging / cloud platform

These are future layers. The MVP targets the highest-value 20%: a developer-friendly simulator + test runner + cheap hardware interface that turns ECU testing from manual work into an automated workflow.

---

## 2. Scope decisions (converged from all idea files)

| Decision | Choice | Rationale |
| --- | --- | --- |
| Positioning | "CI/simulation platform for embedded controllers", not "open CANoe" | Wins on workflow, not features (`competition.txt`) |
| First vertical | EV battery system (BMS + charger + inverter + vehicle controller) | CAN-heavy, safety-related, easy to demo, many startups (`mvp_final.txt` Phase 0) |
| Core language | Rust (tokio, serde, tracing, clap) | Determinism, performance, concurrency, cross-platform, embeddable (`tech_stack.txt`) |
| Config format | YAML, Git-friendly | Reviews, CI, reproducibility (`improvement.txt`) |
| Protocol scope | CAN + CAN FD + DBC only | Smallest useful wedge; ISO-TP/UDS/J1939 later (`tech_stack.txt`) |
| Hardware | SocketCAN only, then one USB-CAN adapter via a `CanInterface` trait | No vendor lock-in; hardware vendors become pluggable backends (`hardware.txt`, `mvp2.txt`) |
| ECU models | YAML-defined config-driven vECUs, then code models | Config first = easy adoption (`difficultiy.txt`) |
| License | AGPLv3 core, Apache-2.0 reference vECUs/examples | Protects ecosystem; models stay user-licensed (`license.txt`) |
| Monetization | Open core + support + enterprise + hardware later | Not built now, but license/architecture must keep the door open (`monetisation.txt`) |

---

## 3. The core workflow (the "killer loop")

```text
git commit
   ↓
build firmware
   ↓
openhil test tests/
   ↓
virtual CAN network + virtual ECUs start
   ↓
500 automated tests run (stimulus → fault injection → assertions)
   ↓
report: PASS/FAIL + timeline + logs
```

Same command locally and in CI (`github.txt`):

```bash
openhil test vehicle.yaml            # developer laptop
openhil test vehicle.yaml --hardware can0   # real ECU on the bench
```

The test does not change between simulation and HIL mode. That is the HIL progression:

- **Run 1**: virtual BMS + virtual motor
- **Run 2**: real BMS + virtual motor
- **Run 3**: real BMS + real motor (`vEcu.txt`)

---

## 4. Architecture

```text
openhil CLI
   │
   ├── core/            event engine, simulation clock, test execution
   ├── can/             CanInterface trait + SocketCAN backend (+ virtual bus)
   ├── simulator/       virtual ECUs, message scheduler, fault injection
   ├── dbc/             DBC parser + signal encode/decode
   ├── test/            YAML test parser, assertions, step runner
   ├── report/          HTML + JUnit/JSON output
   └── models/          reference vECUs (battery, charger, inverter, VCU)
```

Key abstractions (`mvp_final.txt`, `tech_stack.txt`):

```rust
trait Ecu {
    fn update(&mut self, time: Timestamp);
    fn on_message(&mut self, msg: CanFrame);
    fn transmit(&mut self) -> Vec<CanFrame>;
}

trait CanInterface {
    fn send(&self, frame: CanFrame);
    fn recv(&self) -> Option<CanFrame>;
}
```

Simulation is the foundation; GUI/CI/cloud are all just clients of the core runtime.

### Reference vECUs to ship (from `vEcu.txt`)

1. **Battery ECU** — `0x100 BatteryStatus` (voltage, current, SOC, state); behaviors: startup, charging, discharging, fault.
2. **Vehicle Control ECU** — inputs `BatteryStatus`/`BrakeStatus`/`DriverRequest`, output `MotorEnable`; demonstrates logic testing.
3. **Charger ECU** — `ChargeRequest`/`ChargeStatus`; accept charging, reject unsafe conditions, timeout.

---

## 5. YAML test format

```yaml
test: battery_shutdown

setup:
  temperature: 25

steps:
  - set:
      battery.voltage: 500

  - wait: 1s

  - expect:
      signal: inverter.enabled
      equals: false
```

Assertions: `equals`, `greater_than`, `less_than`, `timeout`, `message_received`. Fault injection is a first-class step:

```yaml
  - fault:
      type: can_timeout
      message: motor_status
      duration: 500ms
```

---

## 6. Build plan (6 months, from `mvp_final.txt`)

### Phase 1 — Core runtime (Weeks 1–6)
Event engine, simulation clock, virtual CAN bus, `Ecu` trait, first config-driven vECUs.

**Deliverable:** `openhil simulate battery-demo.yaml` shows a live virtual CAN network.

### Phase 2 — Test framework (Weeks 5–10)
YAML parser, step runner, assertions, reports (HTML + JSON).

**Deliverable:** `openhil test tests/` → `14 PASS / 1 FAIL`.

### Phase 3 — Real hardware (Weeks 10–14)
SocketCAN backend; swap one virtual ECU for a real one; same tests unchanged.

**Deliverable:** `openhil test --hardware can0`.

### Phase 4 — Data collection (Weeks 12–16)
Event recorder: CAN frames, ECU states, test events, firmware version. SQLite storage.

**Deliverable:** reproducible run artifacts (`run_001`, replayable).

### Phase 5 — Analysis (Weeks 16–20)
Timeline viewer, CAN viewer, signal graphs, test-run comparison (detect regressions).

### Phase 6 — Developer workflow (Weeks 18–24)
`openhil init/simulate/test/analyze/report`, standard project layout, GitHub Actions example, packaging a reproducible test artifact (`test-run.zip` with firmware + config + logs + report).

---

## 7. Validation demo (from `hardware_demonstator.txt`)

The "wow demo" = **virtual EV powertrain + one real controller + fault injection + automated report**.

- Laptop runs OpenHIL with virtual battery + charger.
- One STM32 (real motor controller) on CAN.
- Demo: normal run (LED on, motor enabled) → inject overvoltage → motor disables → OpenHIL prints timeline and PASS/FAIL report.
- Live firmware swap shows regression testing: remove the safety check → test goes from PASS to FAIL.

Cost: ~€50–100. Hardware is optional for the MVP, but this single demo proves the whole concept.

---

## 8. Team and effort

A single strong Rust/embedded developer can build the first usable prototype (`mvp_final.txt`). Ideal team: 1 Rust engineer (simulation/CLI/CAN) + 1 embedded engineer (protocols/ECU examples/hardware integration). Frontend engineer optional for the Phase 5 dashboard.

---

## 9. MVP success criteria

> **A user can clone the repo, run one command, see a virtual vehicle come alive, connect a controller, and automatically detect a firmware bug.**

Concrete validation targets (`mvp.txt`):
- Someone uses it on a real project (robotics/EV/university — not OEMs).
- 10–20 regular users/contributors; people submit their own vECUs.
- Companies ask for support or enterprise features.

---

## 10. Roadmap after the MVP

1. **Ecosystem** — vECU package format + registry (`openhil install battery-basic`), WASM plugin support for language-agnostic models (`vEcu.txt`, `eco_system.txt`).
2. **Protocols** — ISO-TP, UDS, J1939, CANopen, LIN (`tech_stack.txt`).
3. **Hardware** — OpenHIL Node (Raspberry Pi/STM32-based) after proven demand (`hardware2.txt`).
4. **Analytics** — anomaly detection, run comparison, AI-assisted failure explanations (`analytics.txt`, `competition.txt`).
5. **Monetization** — support contracts, enterprise features, hosted registry, hardware (`monetisation.txt`).

---

## Key risks

- **Scope creep** — automotive is huge; every protocol is a time sink. Guard with the "not building" list in section 1.
- **Chicken-and-egg** — no models without users, no users without models. Mitigate by shipping 3 reference vECUs and one narrow domain (EV battery) first (`value.txt`).
- **Adoption** — the hard part is not the technology. Validate with the 10-users test before building beyond the MVP (`mvp.txt`).
