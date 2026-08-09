# OpenHIL MVP v2 — 0→1 Execution Plan

A rewrite of `MVP_PLAN.md`. v1 was a good product roadmap but a poor 0→1 plan: it
sequenced 6 phases of *features* when 0→1 requires sequencing *validation*.

**0→1 definition:** find one real user who adopts one narrow workflow, before
building anything more. Everything after that is 1→N and is out of this document.

---

## 1. The single wedge

> **A firmware engineer tests their controller against a virtual CAN network —
> stimulus, fault injection, assertions, report — before the rest of the
> hardware exists.**

That is the whole product for 0→1. Not simulation + testing + hardware + analytics
+ dashboards. One workflow, one user, one job:

> *"I changed BMS logic. I want to know, in under a minute on my laptop, whether
> it still behaves correctly when the charger times out."*

### Named first user

**A firmware engineer at a small EV startup or robotics company** (per `market.txt`,
the easiest beachhead). Concrete job-to-be-done: validate controller state
transitions over CAN before the inverter/charger/battery hardware exists.

If we cannot name and reach this person, we do not start building — we go find one.

---

## 2. The 0→1 slice (what we build)

Smallest thing that demonstrates the workflow. **Pure software. No hardware mode.**

```text
openhil test tests/
   │
   ├── virtual CAN bus
   ├── vECUs: Battery (0x100), Charger, Vehicle Controller
   ├── fault injection (message loss, signal set, delay)
   ├── YAML test runner + assertions
   └── HTML + JSON report
```

That is also the demo. If this works, it is already §7 of v1 — the whole pitch —
in weeks, not months.

### Explicitly cut (1→N, not 0→1)

- SocketCAN / real ECU hardware mode
- SQLite / Parquet storage, event recorder
- Timeline viewer, signal graphs, run comparison
- `openhil init/analyze/report/package`, CI YAML examples
- DBC parser (start with YAML signal defs; add DBC only if a first user asks)
- WASM plugins, model registry, ecosystem tooling
- UDS, ISO-TP, J1939, LIN, CAN FD

---

## 3. Execution sequence (validations, not features)

### Step 0 — Find one user first (before code, 1–2 weeks)

- Write the pitch: *"run automated tests against a virtual CAN network on your
  laptop"* (the good first pitch from `usecase.txt`, not a feature list).
- Recruit **1–3 firmware engineers** from a robotics/EV startup, Formula Student,
  or a university lab. Offer the tool in exchange for a real scenario and feedback.
- Get one *real* scenario in hand: their DBC-ish messages, their state machine,
  their fault they want caught.
- **Kill gate:** no engineer willing to hand over a scenario → stop. The wedge
  is wrong or the target is wrong; do not build in a vacuum.

### Step 1 — Spike the demo (weeks 0–2)

Build the thinnest vertical that runs the *user's* scenario:

- YAML config for ECUs and signals (no DBC yet)
- virtual CAN bus + message scheduler
- `Ecu` trait with config-driven and simple Rust models
- test runner: `send`, `wait`, `expect`, `fault`
- HTML report with a timeline of the failing test

**Goal:** the §7 demo from v1 runs on a laptop — overvoltage injected, motor
disables, report shows PASS/FAIL and *why*.

### Step 2 — Put it in front of the first users (weeks 2–4)

- Port their scenario into the tool; run it against their expectations.
- Watch them use it. Do not ship features from a backlog — collect only the
  pain they hit. Repeat the loop: the features they repeatedly request are the
  roadmap (`mvp.txt`).
- **Kill gate:** by week 4, no user can run their own scenario end-to-end
  (loading, running, interpreting) → the DX or the wedge is wrong. Fix or pivot.

### Step 3 — Harden the loop (weeks 4–8)

Only what the first users demonstrated pain on, in order of request frequency.
Candidates (not promises): DBC import, more assertion types, error messages that
explain *why* a test failed, CLI polish, CI-friendly JSON output.

**Kill gate:** if nobody has used it on a real project by week 8, stop and
reconsider — do not keep building.

---

## 4. 0→1 success criteria (from `mvp.txt`, trimmed)

- One or more engineers use it on a **real project** and report a caught bug or
  saved time — not a "nice demo."
- Someone outside the initial contacts asks for it or contributes a vECU.
- We can name the next 5 users before touching 1→N features.

Not criteria: GitHub stars, contributor count, companies asking for support.
Those are 1→N.

---

## 5. Architecture (unchanged core, kept minimal)

```text
openhil CLI
   ├── core/        simulation clock, event scheduler, test execution
   ├── can/         virtual bus (CanInterface trait; SocketCAN later)
   ├── simulator/   Ecu trait, message scheduler, fault injection
   ├── test/        YAML parser, step runner, assertions
   └── report/      HTML + JSON
```

```rust
trait Ecu {
    fn update(&mut self, time: Timestamp);
    fn on_message(&mut self, msg: CanFrame);
    fn transmit(&mut self) -> Vec<CanFrame>;
}
```

Config-driven vECUs first (easy adoption, per `difficultiy.txt`); code models when
the first user needs behavior a config can't express.

### Reference vECUs shipped in the slice

- **Battery** — `0x100` BatteryStatus (voltage, current, SOC, state); startup / charging / discharging / fault
- **Charger** — ChargeRequest / ChargeStatus; accept, reject unsafe, timeout
- **Vehicle Controller** — BatteryStatus + BrakeStatus + DriverRequest → MotorEnable

---

## 6. Example test

```yaml
test: charger_timeout_safe_state

steps:
  - send: { id: 0x200, signal: charge_request, value: on }
  - wait: 500ms
  - expect: { signal: charger.state, equals: CHARGING }
  - fault:
      type: can_timeout
      message: battery_status
      duration: 500ms
  - expect: { signal: vehicle.motor_enable, equals: false, within: 1s }
```

---

## 7. Team and effort

One strong Rust developer, ~8 weeks. Embedded domain knowledge helps but the
slice is mostly software engineering. Do not add a second person until Step 2
shows demand.

## 8. Risks

- **Building before finding a user** — the #1 risk. Mitigated by Step 0 being a
  gate, not a phase.
- **Scope creep** — every phase of v1 is a temptation. Guard with the cut list
  in §2; features enter only from observed user pain.
- **Wrong first user** — if Step 0 fails, change the audience (university lab,
  Formula Student) before changing the product.

---

## Decision point

At week 8, one of three outcomes:

1. **Adopted** — real user, real bug caught → plan 1→N (hardware mode, DBC, CI).
2. **Close, but friction** — user wants it, DX blocks them → fix the loop, keep going.
3. **No traction** — nobody used it on real work → stop or pivot the wedge; do
   not proceed to hardware/dashboards/ecosystem.
