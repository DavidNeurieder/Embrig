# OpenHIL

Deterministic hardware-in-the-loop testing for CAN networks. Describe your bus
with a DBC file and a `vehicle.yaml` (config-driven nodes, or your own Rust
ECUs), write test suites as plain YAML, and run the **same suite** against a
built-in virtual simulation or a real CAN interface via SocketCAN.

The bundled EV powertrain example needs no CAN hardware to get started:

```sh
cargo run --bin openhil -- test examples/ev-powertrain/vehicle.yaml
```

→ `5 passed, 0 failed` (and the README in that directory shows how to flip one
test to FAIL).

## What it does

A Cargo workspace with six crates:

| Crate | Purpose |
| --- | --- |
| `openhil-core` | deterministic virtual CAN simulation: `CanFrame`, integer-µs clock, `Ecu` trait, routing, fault injection, event recorder |
| `openhil-dbc` | DBC parser (`BO_`/`SG_`/`VAL_`) + signal codec (Intel/Motorola, signed, factor/offset) |
| `openhil-models` | vehicle YAML config, config-driven ECUs, reference EV vECUs (charger/VCU/motor) used by the bundled example |
| `openhil-can` | async SocketCAN backend (feature `socketcan`) |
| `openhil-test` | YAML test DSL, runner (virtual + hardware targets), HTML/JSON reports |
| `openhil-cli` | the `openhil` binary: `init` / `simulate` / `test` / `report` |

## Concepts

- **Bus from DBC** — any message map parses `BO_`/`SG_`/`VAL_` with Intel or
  Motorola byte order, signed values, factor/offset scaling and symbolic value
  tables. Assertions decode real signals, never raw bytes.
- **Nodes** — ECUs are either config-driven (signal values in YAML, no code)
  or custom Rust ECUs implementing the `Ecu` trait.
- **One suite, two targets** — the same YAML tests run against the virtual
  simulation or a live SocketCAN bus. `set_signal` and `fault` need the virtual
  router and are rejected with a clear error on real hardware rather than
  silently ignored.
- **Determinism** — the simulation steps on an integer-microsecond clock with
  ECUs in config order, so a PASS→FAIL flip is always caused by your change —
  never by the runner.

## CLI

```sh
# Scaffold a new project from the bundled EV powertrain templates (vehicle.yaml, powertrain.dbc, tests/)
openhil init my-project

# Run the virtual simulation and print the bus trace
openhil simulate my-project/vehicle.yaml --duration 2s --verbose

# Run YAML tests (defaults to the `tests` directory next to vehicle.yaml)
openhil test my-project/vehicle.yaml
openhil test my-project/vehicle.yaml tests/overvoltage.yaml
openhil test my-project/vehicle.yaml --report report.html

# Render a stored JSON result to HTML/JSON
openhil report report.json --format html --output report.html
```

Exit codes: `0` all tests pass · `1` test failures · `2` usage/config/load errors.

### Hardware mode

`test` picks a target from the `interfaces:` section of `vehicle.yaml`:

```sh
cargo build --features socketcan
openhil test my-project/vehicle.yaml --interface vcan0
```

The same YAML tests then drive a real CAN bus instead of the virtual one.
`set_signal` and `fault` steps are virtual-only (no router exists on a real
bus) and are rejected with a clear error rather than silently ignored. The
`--interface` flag maps to an interface **name** in `vehicle.yaml` (e.g.
`virtual` or `vcan0`); the concrete device is the `interface:` field on the
`socketcan` entry.

### Example: EV powertrain

`examples/ev-powertrain/` is an included demo you can build on: a virtual EV
powertrain (charger, VCU, motor) on a DBC-defined bus, with YAML test suites
and a step-by-step walkthrough. See `examples/ev-powertrain/README.md` for the
student walkthrough and a bill of materials (~€0 software, ~€15–30 with one
real STM32 ECU).

## Development

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --features socketcan -- -D warnings
cargo test --workspace --features socketcan
```

If you have a real or virtual CAN device:

```sh
scripts/vcan-smoke.sh     # brings up vcan0 (needs sudo) and runs the socketcan path
```

Website: <https://davidneurieder.github.io/openhil/>. See [`CHANGELOG.md`](CHANGELOG.md) for release history.

## License

AGPL-3.0-or-later
