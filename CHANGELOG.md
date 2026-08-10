# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Nothing yet.

## [0.1.0] - 2026-08-10

Initial release: deterministic hardware-in-the-loop CAN testing for students
and engineers, running the same YAML tests against a virtual simulation or a
real SocketCAN bus.

### Added

- **`openhil-core`** — deterministic virtual CAN simulation: `CanFrame`,
  integer-microsecond clock, `Ecu` trait, config-ordered routing, fault
  injection (drop/delay/corrupt), and an event recorder. Fully reproducible:
  a PASS→FAIL flip is always caused by your change, never the runner.
- **`openhil-dbc`** — DBC parser (`BO_`/`SG_`/`VAL_`) and signal codec
  (Intel/Motorola byte order, signed values, factor/offset scaling, symbolic
  `VAL_` tables).
- **`openhil-models`** — vehicle YAML configuration, config-driven ECUs, and
  reference EV vECUs (charger, VCU, motor) with an overvoltage/safe-state
  behavior set.
- **`openhil-can`** — async SocketCAN backend (`socketcan` feature), including
  own-message reception for single-socket loopback verification.
- **`openhil-test`** — YAML test DSL (`send`, `set_signal`, `wait`, `expect`,
  `fault`), assertion operators (`equals`, `greater_than`, `less_than`,
  `present`, `absent`), durations in `us`/`ms`/`s`, a per-test timeout budget,
  an async runner with virtual and hardware targets, and HTML/JSON reports.
  `set_signal`/`fault` are rejected with a clear error on hardware targets.
- **`openhil-cli`** — the `openhil` binary: `init` (project scaffolding),
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

[Unreleased]: https://github.com/openhil/openhil
[0.1.0]: https://github.com/openhil/openhil/releases/tag/v0.1.0
