# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Project renamed **OpenHIL → Embrig** ("embrig — open embedded testing");
  crates renamed `openhil-*` → `embrig-*`, the binary is `embrig`, and the
  repository/website URLs now use `embrig`. `MVP_PLAN*.md` remain as historical
  artifacts.

### Added

- **Software-in-the-loop (SIL)** — run host-compiled firmware against the
  virtual bus with the same YAML suites as virtual/hardware targets:
  - **`embrig-sil`** — `SilRegistry` (firmware factories keyed by ECU name,
    closures or `EcuFactory` impls), `SilTarget` (a `TestTarget` that re-runs
    firmware factories on every reset so state never leaks between tests),
    and the `sil_run` helper.
  - **Wall-clock step budget** — each simulated firmware step runs under a
    budget (default 100 ms, override `step_budget_us` in YAML); an overrun
    fails the test with `TargetError::SutTimeout` instead of hanging it.
  - **SUT signal overrides rejected** — the firmware under test is driven via
    the bus, not via `set_signal` (`TargetError::UnsupportedOnSut`); faults
    and config-node signal overrides behave exactly as in virtual mode.
  - `embrig-models` — `EcuKind::Sil` config node and the `EcuFactory` hook on
    `build_simulation_indexed_with`; `type: sil` without a registered factory
    fails with a clear "no firmware registered" error.
  - **`crates/embrig-sil/examples/`** — thermal-controller demo (fixtures +
    suites + `sil_firmware` example) that passes `2/2`.
  - **`crates/embrig-sil/examples/robot/`** — robotics demo: a
    differential-drive rover whose motion controller is SIL firmware, with
    suites for driving on joystick command, e-stop halt, e-stop-release
    resume, and over-speed refusal (`robot_sil` example, passes `4/4`).
  - CLI: `--interface sil` exits 2 with guidance to use `embrig-sil`.
  - Tests: 6 SIL unit tests (suite pass, budget overrun, SUT rejection, faults,
    factory re-run on reset, unknown-registry error) + 1 CLI test.
  - **Guides** — `how_to/how-to-sil-test.md` (wrap existing Rust firmware as the
    system under test) and `how_to/how-to-hil-test.md` (run the same suites
    against a real ECU over SocketCAN), linked from the README and the website.

### Changed

- `embrig-test::TargetError` gains `UnsupportedOnSut` and `SutTimeout`.
- `embrig-core::EcuError` gains `NotRegistered` for unregistered SIL firmware.
- Test count updated in docs (91 total, incl. 10 CLI black-box).

## [0.1.0] - 2026-08-10

Initial release: deterministic hardware-in-the-loop CAN testing for students
and engineers, running the same YAML tests against a virtual simulation or a
real SocketCAN bus.

### Added

- **`embrig-core`** — deterministic virtual CAN simulation: `CanFrame`,
  integer-microsecond clock, `Ecu` trait, config-ordered routing, fault
  injection (drop/delay/corrupt), and an event recorder. Fully reproducible:
  a PASS→FAIL flip is always caused by your change, never the runner.
- **`embrig-dbc`** — DBC parser (`BO_`/`SG_`/`VAL_`) and signal codec
  (Intel/Motorola byte order, signed values, factor/offset scaling, symbolic
  `VAL_` tables).
- **`embrig-models`** — vehicle YAML configuration, config-driven ECUs, and
  reference EV vECUs (charger, VCU, motor) with an overvoltage/safe-state
  behavior set.
- **`embrig-can`** — async SocketCAN backend (`socketcan` feature), including
  own-message reception for single-socket loopback verification.
- **`embrig-test`** — YAML test DSL (`send`, `set_signal`, `wait`, `expect`,
  `fault`), assertion operators (`equals`, `greater_than`, `less_than`,
  `present`, `absent`), durations in `us`/`ms`/`s`, a per-test timeout budget,
  an async runner with virtual and hardware targets, and HTML/JSON reports.
  `set_signal`/`fault` are rejected with a clear error on hardware targets.
- **`embrig-cli`** — the `embrig` binary: `init` (project scaffolding),
  `simulate` (deterministic trace), `test` (virtual or `--interface <name>`
  hardware), and `report` (render stored JSON to HTML/JSON). Exit codes:
  `0` all pass · `1` test failures · `2` usage/config/load errors.
- **`examples/ev-powertrain`** — student walkthrough with BOM, 5 passing
  YAML scenarios, and a documented PASS→FAIL regression proof.
- **Continuous integration** — fmt/clippy/test checks, a SocketCAN feature
  build, an end-to-end example run, and a `vcan0` smoke test
  (`scripts/vcan-smoke.sh`) exercising the real send→receive hardware path.
- **Automated tests** — 81 unit tests across the crates plus 9 CLI black-box
  tests (scaffolding, exit codes, reports, interface resolution, trace output).
- **Root `README.md`** and `IMPLEMENTATION_PLAN.md` documenting the design,
  CLI, DSL, and message map.

[Unreleased]: https://github.com/DavidNeurieder/embrig
[0.1.0]: https://github.com/DavidNeurieder/embrig/releases/tag/v0.1.0
