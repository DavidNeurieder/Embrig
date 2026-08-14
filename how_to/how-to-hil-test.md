# How to run a HIL test on your existing firmware

Hardware-in-the-loop (HIL) runs the same YAML suites against your **real ECU on
a real CAN bus**. Embrig runs on the host, connected to the bus through a
SocketCAN interface; it injects stimulus frames and asserts on the ECU's real
responses. There is no software router in the loop — the device is the
simulation.

This guide assumes your firmware already exists and is flashed onto the device
you want to test.

---

## 1. Build with SocketCAN support

```sh
cargo build --release --features socketcan
```

Use the built binary at `target/release/embrig` (or `cargo run --features
socketcan --bin embrig -- ...`). Without the `socketcan` feature the CLI still
works, but rejects `--interface`.

## 2. Hardware bring-up

### 2a. Choose a CAN interface on the host

- **USB-CAN adapter** (e.g. CANable, PEAK, Kvaser) — simplest; appears as
  `can0` under Linux.
- **Raspberry Pi + CAN HAT** — uses the same SocketCAN stack.
- **Built-in/PCI CAN** controller (e.g. on some industrial boards).

Check the device is visible:

```sh
ip link show can0
```

If it does not appear, load the driver module (`modprobe <driver>`) and bring it
up:

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

> **Bitrate must match your ECU's** (500 kbit/s is common). A mismatch means
> error frames and no communication.

### 2b. Wire the bus

- Connect **CANH to CANH** and **CANL to CANL** between the host interface and
  every ECU.
- Share a common **GND** between all nodes.
- Place a **120 Ω termination resistor** at **each end** of the bus (a real
  network has exactly two — no more, no less).
- Power the ECU(s) under test.

### 2c. Optional: smoke-test the socket path without hardware

If you only want to prove the SocketCAN path end to end (not the ECU), a virtual
interface works:

```sh
sudo scripts/vcan-up.sh      # creates and brings up vcan0 (idempotent)
sudo scripts/vcan-down.sh    # tear it down again
```

This is a loopback check, not a HIL test — run `scripts/vcan-smoke.sh` to see it
working.

## 3. Point Embrig at the bus: DBC + `vehicle.yaml`

### 3a. The DBC file

Same as everywhere: the message map of your real network. `expect` decodes the
ECU's real frames through it. Use the DBC your firmware's CAN stack was built
from.

### 3b. `vehicle.yaml`

The hardware target needs the `interfaces` entry; the `ecus` list is only used
by the virtual/SIL simulation and can stay empty (or document the network):

```yaml
name: ev-powertrain
dbc: powertrain.dbc

interfaces:
  - name: can0
    type: socketcan
    interface: can0   # the concrete device name from step 2
```

To also run a **simulated rest bus** next to the real ECU, use `type: restbus`
and list the rest-bus nodes (config nodes and/or the reactive vECUs) under
`ecus` as usual — see [§4b](#4b-rest-bus-on-hil-simulated-peers). The same
`ecus` blocks you already write for virtual/SIL become the rest bus on HIL:

```yaml
name: ev-powertrain
dbc: powertrain.dbc

ecus:
  - name: battery
    type: config
    message: BatteryStatus
    period_us: 100000
    signals:
      voltage: 400.0
      state: "READY"

interfaces:
  - name: hil
    type: restbus
    interface: can0
```

## 4. Write `send`-based suites

On real hardware with the bare `socketcan` target there is no virtual router,
so:

- **Config nodes do not transmit.** Periodic frames, `set_signal` overrides and
  `fault` injection are virtual-only — the runner rejects them with
  `UnsupportedOnHardware` rather than ignoring them.
- **Stimulus is explicit `send` steps** — raw frames with byte lists (max 8
  bytes). The ECU under test does the rest.
- **Assertions are `expect` steps** — they decode the ECU's real frames through
  the DBC, exactly as in virtual mode.

A HIL test therefore looks like: send the input frame(s), wait for the ECU's
response, assert on it:

```yaml
name: motor_enables_on_battery_ready
timeout: 5s
steps:
  - send: { id: 0x100, data: [0xA0, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00] }
  - expect: { id: 0x220, signal: motor_enable, equals: true, within: 1s }
  - expect: { id: 0x230, signal: state, equals: "RUNNING", within: 1s }
```

> **If you want the rest bus back** (config nodes transmitting, `set_signal`,
> faults, reactive vECUs), use the `restbus` interface instead — see
> [§4b](#4b-rest-bus-on-hil-simulated-peers). `expect` assertions are identical
> in both modes.

### How to get the raw bytes

Encoding DBC signals to raw bytes by hand is error-prone. Two easy paths:

1. **Run the same scenario in the virtual simulation first** and read the bytes
   the config nodes emit:

   ```sh
   embrig simulate my-project/vehicle.yaml --duration 2s --verbose
   ```

   Each line is `ts 0xID [len] XX XX XX ...` — copy the `XX` bytes into your
   `send` step. This also validates the DBC.

2. **Compute from the DBC**: physical value ÷ factor = raw integer, stored
   little-endian at the signal's bit offset. (For `voltage` on `0|16@1+
   (0.1,0)`, 400.0 V → raw 4000 → bytes `A0 0F`.)

## 4b. Rest bus on HIL (simulated peers)

With `type: restbus` the `ecus` list from your vehicle.yaml runs as a
**simulated rest bus** on the same interface as the real ECU under test:

- Config nodes transmit their frames **on the wire** at their `period_us` — so
  the DUT receives the same periodic telemetry it would on a real network.
- `set_signal` on a config node overrides its next transmission (the DUT sees
  the new value on the bus).
- `fault` steps drop/delay/corrupt the **rest bus's** frames before they reach
  the wire (e.g. simulate the battery dropping off the network).
- Reactive vECUs (e.g. `motor`, `charger`) listen on the wire: send a real
  frame and the vECU's response is transmitted on the bus for the DUT — or
  assert on it yourself.
- `expect` sees **everything on the bus**: rest-bus frames and the DUT's real
  responses, through the same DBC decoder.

The sim clock chases the host's clock, so periods land in wall-clock time.
There is **no microsecond lockstep** — emission is accurate to the drive loop's
poll interval (`POLL_US`), so keep `within` generous enough for host jitter,
as on bare hardware.

Same suites, rest bus or not: only the interface and (optionally) `set_signal`
change.

```sh
embrig test my-project/vehicle.yaml --interface hil
```

What still does **not** work on the rest bus:

- **Faults on the DUT's own frames.** There is still no router on the live bus;
  `fault` only affects frames the simulated rest bus emits.
- **Reset of the DUT.** `reset` rebuilds the rest bus between tests, not the
  real ECU — each test must still drive the DUT into a known state.

## 5. Sanity check: loopback

Before pointing at the ECU, verify the socket round trip with the bundled
loopback test — it sends one frame and expects it back on the same socket
(own-message reception is enabled):

```sh
embrig test examples/ev-powertrain/vehicle.yaml scripts/loopback.yaml --interface can0
```

or run it as a pre-flight before a real suite — `--check` runs the same loopback
and aborts with a non-zero exit if the round trip fails:

```sh
embrig test my-project/vehicle.yaml --interface can0 --check
```

→ `1 passed, 0 failed` (or `CHECK bus loopback ... PASS`) means send→receive
works on the interface.

## 6. Run the suite against the ECU

```sh
embrig test my-project/vehicle.yaml --interface can0
```

or, to pick specific suites (files or directories, relative to cwd or the
vehicle.yaml directory):

```sh
embrig test my-project/vehicle.yaml tests/ --interface can0
embrig test my-project/vehicle.yaml my-suite.yaml --interface can0
```

Or drive the same path from a runnable example (handy without a terminal-facing
CLI install, and the fastest way to prove the stack on a `vcan` bus):

```sh
cargo run --example can_hil --package embrig-test --features socketcan
```

It defaults to the `ev-powertrain` fixture on the first `socketcan` interface in
`vehicle.yaml` (or `vcan0`) and runs the bundled loopback spec — virtual-only
suites (`set_signal`, faults, config nodes) do not run on hardware. Pass
`INTERFACE VEHICLE [TEST...]` to override; point `TEST` at your own send-based
suites:

```sh
cargo run --example can_hil --package embrig-test --features socketcan -- can0 my-project/vehicle.yaml tests/
```

`--interface` also accepts a raw kernel interface name that is not declared in
`vehicle.yaml` — it is used directly as the SocketCAN device.

Results go to stdout (`PASS`/`FAIL` per test, then totals) and the exit code is
non-zero on any failure. For a report:

```sh
embrig test my-project/vehicle.yaml --interface can0 --report report.html
embrig test my-project/vehicle.yaml --interface can0 --report report.json --report-format json
```

## 6b. Converting a SIL suite to HIL

SIL and HIL share the same `expect` assertions. On the **bare `socketcan`**
target the only mechanical change is replacing config-node stimulus with
explicit `send` frames (real hardware has no config nodes transmitting for
you). With the **`restbus`** target most suites need no conversion at all — the
config nodes, `set_signal` and faults from the SIL suite work unchanged; only
SIL firmware nodes (compiled-for-host ECUs) have no HIL equivalent.

For the bare `socketcan` target, for each virtual-only step:

| Virtual / SIL step            | HIL replacement                                                                 |
| ----------------------------- | ------------------------------------------------------------------------------- |
| `set_signal` on a config node | `send` the pre-encoded raw frame; re-send whenever the value must change        |
| config-node periodic frames   | drop — nothing transmits on its own; your `send` steps are the stimulus         |
| `fault` injection             | drop — no virtual router; inject faults in firmware or in the harness instead   |
| `expect`                      | unchanged                                                                        |
| `wait`                        | unchanged                                                                        |

To encode a signal to raw bytes without doing the math by hand, run the same
scenario in the virtual simulation and read the frames it emits:

```sh
embrig simulate my-project/vehicle.yaml --duration 2s --verbose
```

The bundled rover example ships send-based HIL twins of its SIL suites in
`crates/embrig-sil/examples/robot/suites_hil/` — compare a twin against its
original in `suites/` to see the transform applied. Note that a SIL harness may
have config nodes fighting injected frames on the same ids (the twins' `resume`
case is one example); on real hardware nothing else transmits, so the `send`
stimulus persists.

## 7. What is different from virtual/SIL mode

- **Wall-clock timing.** `wait` sleeps real time and `expect` polls the bus for
  up to `within`. Tests are no longer bit-for-bit deterministic — keep `within`
  generous enough for your bus jitter and hardware boot times.
- **`reset` does not reset the ECU.** The bare `socketcan` target has nothing
  to reset; the `restbus` target rebuilds its simulated rest bus between tests.
  Either way the device itself keeps its state, so each test must be
  self-contained: drive the ECU into a known state at the start of every test.
  The device must tolerate being re-run.
- **Own-message reception on.** Frames you send are also seen by your own
  socket (that is what the loopback test relies on).
- **The bus is live.** Any other node transmitting on the same interface shows
  up in `expect`. Use a dedicated test bus, not a production network.

## Caveats

- **No router for the DUT's frames.** On the bare `socketcan` target
  `set_signal`/`fault`/config-node transmission are rejected with a clear
  error. On the `restbus` target they work, but only on the simulated rest bus
  — faults and signal overrides can never touch the real ECU's own frames. To
  test the DUT's fault handling, add fault-injection to your firmware or rig
  the harness yourself.
- **Isolation.** Run against a dedicated bus with only the device(s) under
  test. Do not point this at a production CAN network.
- **Timing margins.** First boot, watchdog resets and host scheduling all add
  real-time delay — assert on outcomes within a window, not on exact timing.
  The `restbus` emission cadence is host-granular (poll-interval tick), not a
  hard real-time guarantee.

## Next: start SIL-first

Most teams develop suites in [SIL](how-to-sil-test.md) (deterministic, no
hardware), then run the same tests on HIL once they are stable — switching only
the interface and replacing `set_signal` stimulus with `send` frames.
