# Embrig

Deterministic embedded testing for CAN and Ethernet (UDP) networks — from
software-in-the-loop (SIL) to hardware-in-the-loop (HIL). Describe your bus
with a DBC file and a `vehicle.yaml` (config-driven nodes, or your own Rust
ECUs), write test suites as plain YAML, and run the **same suite** against
host-compiled firmware (SIL), a built-in virtual simulation, or a real CAN
interface via SocketCAN — and against a virtual or real Ethernet (UDP) network
for nodes that talk IP.

The bundled EV powertrain example needs no CAN hardware to get started:

```sh
cargo run --bin embrig -- test examples/ev-powertrain/vehicle.yaml
```

→ `5 passed, 0 failed` (and the README in that directory shows how to flip one
test to FAIL).

## What it does

A Cargo workspace with eight crates:

| Crate | Purpose |
| --- | --- |
| `embrig-core` | deterministic virtual CAN simulation: `CanFrame`, integer-µs clock, `Ecu` trait, routing, fault injection, event recorder |
| `embrig-dbc` | DBC parser (`BO_`/`SG_`/`VAL_`) + signal codec (Intel/Motorola, signed, factor/offset) |
| `embrig-models` | vehicle YAML config, config-driven ECUs, reference EV vECUs (charger/VCU/motor) used by the bundled example |
| `embrig-net` | deterministic virtual Ethernet (UDP) network: netmap codec, datagrams, `UdpEcu` trait, routing, fault injection |
| `embrig-can` | async SocketCAN backend (feature `socketcan`) |
| `embrig-sil` | software-in-the-loop: run host-compiled firmware against the virtual bus, wall-clock step budgets, `sil_run` helper |
| `embrig-test` | YAML test DSL, runner (virtual + SIL + UDP + hardware targets), HTML/JSON reports |
| `embrig-cli` | the `embrig` binary: `init` / `simulate` / `test` / `report` |

## Concepts

- **Bus from DBC** — any message map parses `BO_`/`SG_`/`VAL_` with Intel or
  Motorola byte order, signed values, factor/offset scaling and symbolic value
  tables. Assertions decode real signals, never raw bytes.
- **Netmap for Ethernet** — a UDP network is described by a netmap instead of
  a DBC: messages keyed by destination endpoint, with named fields at byte
  offsets (`u8`, `bool`, little/big-endian integers, `f32le`/`f64le`, scaling
  and symbolic values). Same DSL, same determinism, same fault injection —
  just on IP.
- **Nodes** — ECUs are either config-driven (signal values in YAML, no code)
  or custom Rust ECUs implementing the `Ecu` (CAN) or `UdpEcu` (UDP) trait.
  With SIL, the node under test is your **real firmware**, compiled for the
  host.
- **One suite, many targets** — the same YAML tests run against host-compiled
  firmware (SIL), the virtual simulation, a live SocketCAN bus, or a UDP
  network (virtual or a real socket). `set_signal`, `set_field` and `fault`
  need the virtual router and are rejected with a clear error on real hardware
  rather than silently ignored; on a SIL target, driving the firmware itself
  is likewise rejected — you test it through the bus.
- **Determinism** — the simulation steps on an integer-microsecond clock with
  ECUs in config order, so a PASS→FAIL flip is always caused by your change —
  never by the runner.

## CLI

```sh
# Scaffold a new project from the bundled EV powertrain templates (vehicle.yaml, powertrain.dbc, tests/)
embrig init my-project

# Run the virtual simulation and print the bus trace
embrig simulate my-project/vehicle.yaml --duration 2s --verbose

# Run YAML tests (defaults to the `tests` directory next to vehicle.yaml)
embrig test my-project/vehicle.yaml
embrig test my-project/vehicle.yaml tests/overvoltage.yaml
embrig test my-project/vehicle.yaml --report report.html

# Render a stored JSON result to HTML/JSON
embrig report report.json --format html --output report.html
```

Exit codes: `0` all tests pass · `1` test failures · `2` usage/config/load errors.

### Hardware mode

`test` picks a target from the `interfaces:` section of `vehicle.yaml`:

```sh
cargo build --features socketcan
embrig test my-project/vehicle.yaml --interface vcan0
```

The same YAML tests then drive a real CAN bus instead of the virtual one.
`set_signal` and `fault` steps are virtual-only (no router exists on a real
bus) and are rejected with a clear error rather than silently ignored. The
`--interface` flag maps to an interface **name** in `vehicle.yaml` (e.g.
`virtual`, `vcan0` or `udp`); the concrete device is the `interface:` field on
the `socketcan` entry.

### UDP mode (Ethernet)

Nodes that talk UDP/IP (not CAN) use a netmap instead of a DBC. In
`vehicle.yaml`, declare `eth_ecus:` (config-driven nodes, or `type: udp-sil`
for host-compiled firmware) plus a `networks:` entry pointing at the netmap,
then run the same YAML suites with `--interface udp`:

```sh
cargo run --example udp_rover --package embrig-test
```

→ `4 passed, 0 failed`. The example rover in
`crates/embrig-test/examples/rover/` shows the layout: `netmap.yaml` maps
message names to fields and endpoints, `vehicle.yaml` declares the Ethernet
nodes and host, and `suites/*.yaml` assert on decoded fields. UDP suites add
four steps that mirror the CAN ones: `send_udp`, `set_field`, `expect_udp`
and `fault_udp` (drop / delay / corrupt). A vehicle can be pure-Ethernet — its
`dbc:` field is simply omitted.

### SIL mode (software-in-the-loop)

SIL runs your actual firmware — compiled for the host — against the virtual
bus, so suites exercise real control logic without hardware. The firmware is
an `Ecu` implementation registered by ECU name:

```sh
cargo run --example sil_firmware --package embrig-sil
```

→ `2 passed, 0 failed`

The example in `crates/embrig-sil/examples/` is a thermal-controller:
`fixtures/vehicle.yaml` declares a `sensor` config node and a `controller`
node with `type: sil` (firmware is code, not config). `SilRegistry` binds the
node name to the firmware, `sil_run` runs the YAML suites, and each simulated
firmware step runs under a wall-clock budget (default 100 ms, override with
`step_budget_us`) — an overrun fails the test instead of hanging it. The CLI
does not execute `--interface sil`; SIL is used from the crate:

```rust
let mut registry = SilRegistry::new();
registry.register("controller", |_, _| Ok(Box::new(ControllerFirmware::new())));
let result = sil_run(&config, &dbc, registry, &suites)?;
```

A second example applies the same pattern to robotics — a differential-drive
rover whose motion controller is SIL firmware (`crates/embrig-sil/examples/robot/`):

```sh
cargo run --example robot_sil --package embrig-sil
```

→ `4 passed, 0 failed`: joystick commands drive the wheels, the e-stop halts
(and releasing it resumes), and commands above the 1.5 m/s limit are refused.

### Example: EV powertrain

`examples/ev-powertrain/` is an included demo you can build on: a virtual EV
powertrain (charger, VCU, motor) on a DBC-defined bus, with YAML test suites
and a step-by-step walkthrough. See `examples/ev-powertrain/README.md` for the
student walkthrough and a bill of materials (~€0 software, ~€15–30 with one
real STM32 ECU).

## Guides

Step-by-step walkthroughs for testing an existing embedded system:

- [`how_to/how-to-sil-test.md`](how_to/how-to-sil-test.md) — run your Rust
  firmware as the system under test on the virtual bus: DBC + `vehicle.yaml`,
  wrapping the firmware in the `Ecu` trait, YAML suites, `SilRegistry` +
  `sil_run`.
- [`how_to/how-to-hil-test.md`](how_to/how-to-hil-test.md) — run the same
  suites against a real ECU on a real CAN bus: SocketCAN build, hardware
  bring-up, `send`-based stimulus, loopback sanity check, caveats.
- [`how_to/how-to-udp-test.md`](how_to/how-to-udp-test.md) — test Ethernet
  (UDP) nodes: netmap instead of DBC, `eth_ecus` + `networks` in
  `vehicle.yaml`, the `send_udp` / `expect_udp` / `fault_udp` steps, and
  running against a real socket.

## Development

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --features socketcan -- -D warnings
cargo test --workspace --features socketcan
cargo run --example sil_firmware --package embrig-sil
cargo run --example robot_sil --package embrig-sil
cargo run --example udp_rover --package embrig-test
```

If you have a real or virtual CAN device:

```sh
scripts/vcan-smoke.sh     # brings up vcan0 (needs sudo) and runs the socketcan path
```

Website: <https://davidneurieder.github.io/embrig/>. See [`CHANGELOG.md`](CHANGELOG.md) for release history.

## License

AGPL-3.0-or-later
