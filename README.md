# OpenHIL

Deterministic hardware-in-the-loop CAN testing for students and engineers. Run
a virtual EV powertrain on any laptop, inject faults, get a report that
explains each failure — then run the **same YAML tests** against a real ECU
over SocketCAN.

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
| `openhil-models` | vehicle YAML config, config-driven ECUs, reference EV vECUs (charger/VCU/motor) |
| `openhil-can` | async SocketCAN backend (feature `socketcan`) |
| `openhil-test` | YAML test DSL, runner (virtual + hardware targets), HTML/JSON reports |
| `openhil-cli` | the `openhil` binary: `init` / `simulate` / `test` / `report` |

The simulation is fully deterministic: ECUs step in config order on an integer
microsecond clock, so a PASS→FAIL flip is always caused by your change — never
by the runner.

## CLI

```sh
# Scaffold a new vehicle project (vehicle.yaml, powertrain.dbc, tests/)
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

See `examples/ev-powertrain/README.md` for the student walkthrough and a bill
of materials (~€0 software, ~€15–30 with one real STM32 ECU).

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

## License

AGPL-3.0-or-later
