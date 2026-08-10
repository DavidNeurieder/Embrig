# How to test Ethernet (UDP) nodes

Embrig's second transport is Ethernet (UDP/IP). Instead of a DBC, a UDP network
is described by a **netmap**: each message is identified by its destination
endpoint, with named fields at byte offsets. The same YAML suite format, the
same determinism, the same fault injection — just over IP.

A vehicle can be pure-Ethernet (no `dbc:` field) or mixed with CAN nodes; the
CLI picks the transport from the `--interface` name in `vehicle.yaml`.

---

## 1. Describe the network: `netmap.yaml`

Fields are named values at byte offsets inside the payload. The supported field
types are `u8`, `bool`, `u16le`/`u16be`, `u32le`/`u32be`, `i16le`, `i32le`,
`f32le`, `f64le` — each optionally scaled with `factor`/`shift`, and optionally
mapped to symbolic values:

```yaml
messages:
  DriveCommand:
    dst: 192.168.1.30:5000
    length: 8
    fields:
      forward: { offset: 0, type: f32le }
      estop:   { offset: 4, type: bool }
  MotionState:
    dst: 192.168.1.10:5000
    length: 8
    fields:
      speed: { offset: 0, type: f32le }
      state:
        offset: 4
        type: u8
        values:
          0: STOPPED
          1: DRIVING
          2: EMERGENCY
```

`dst` is the endpoint the message is delivered to — on a real link it is the
destination IP:port, in the virtual simulation it is the routing key (exactly
how a CAN message id routes frames). Everything in the suites refers to the
message by **name**, never by endpoint.

## 2. Declare the nodes: `vehicle.yaml`

```yaml
name: rover
eth_ecus:
  - name: joystick
    type: udp-config
    address: 192.168.1.20:6000
    message: DriveCommand
    period_us: 20000
  - name: motion
    type: udp-config
    address: 192.168.1.30:5000
    message: MotionState
    period_us: 50000
    fields:
      speed: 0.0
      state: STOPPED
networks:
  - name: eth
    type: udp
    host: 192.168.1.10:5000
    netmap: netmap.yaml
interfaces:
  - name: udp
    type: udp
```

- **`type: udp-config`** nodes transmit their netmap message on `period_us`,
  with the `fields:` values (scaled/symbolic values supported) — the Ethernet
  equivalent of a config ECU, no code needed.
- **`type: udp-sil`** nodes are host-compiled firmware implementing the
  `UdpEcu` trait (see `crates/embrig-test/src/udp.rs` and the `udp_run_with_firmware`
  helper); like CAN SIL nodes they are registered by name and cannot be driven
  by `set_field` — you test them through the network.
- **`networks:`** lists the host endpoint (`host`) the test bench binds/addresses
  and the netmap file, relative to `vehicle.yaml`. `host` is the source address
  of injected datagrams and the destination of telemetry.
- **`interfaces:`** lets `--interface udp` select this target. A vehicle with no
  `dbc:` field is pure-Ethernet and skips DBC loading entirely.

## 3. Write UDP suites

The four extra steps mirror the CAN ones:

```yaml
name: motion_state_reports_stopped
timeout: 5s
steps:
  - wait: { time: 60ms }                                   # let telemetry flow
  - expect_udp: { message: MotionState, field: state, equals: "STOPPED", within: 1s }
  - set_field: { ecu: motion, message: MotionState, field: speed, value: 3.0 }
  - wait: { time: 60ms }
  - expect_udp: { message: MotionState, field: speed, equals: 3.0, within: 1s }
```

- **`send_udp`** — inject a datagram from the host to the message's `dst`:
  `send_udp: { message: DriveCommand, fields: { forward: 2.0, estop: true } }`
- **`set_field`** — override one field on a `udp-config` node's next transmit.
- **`expect_udp`** — assert on a decoded field (`equals`, `greater_than`,
  `less_than`, symbolic values) or on presence: `present: false` (alias
  `absent: true`) fails as soon as a matching datagram is seen.
- **`fault_udp`** — virtual-network fault injection by message name:
  - `type: drop` — suppress the message for `duration`
  - `type: delay` — hold datagrams back (`delay`) for `duration`
  - `type: corrupt` — flip a payload byte (`byte`, `mask`)

The reference suites live in `crates/embrig-test/examples/rover/suites/`.

## 4. Run it

```sh
cargo run --example udp_rover --package embrig-test
```

→ `4 passed, 0 failed` (virtual network). From the CLI, with a `udp` interface
in `vehicle.yaml`:

```sh
embrig test my-project/vehicle.yaml --interface udp
embrig test my-project/vehicle.yaml --interface udp --report report.html
```

Reports, exit codes and test-file resolution behave exactly like the CAN path.

## 5. Real hardware: `UdpHardwareTarget`

Talking to a real Ethernet link is like CAN HIL: the target binds the `host`
endpoint as a UDP socket and forwards to real endpoints. As with SocketCAN,
there is no virtual router, so `set_field` and `fault_udp` are rejected with a
clear `UnsupportedOnHardware` error; stimulus is explicit `send_udp` and
assertions are `expect_udp`. In code:

```rust
use embrig_test::UdpHardwareTarget;
let target = UdpHardwareTarget::new(host, netmap).await?;
let suite = embrig_test::run_suite(&mut target, &suites, "udp-hil").await?;
```

Use a dedicated network segment with only the devices under test, and keep
`within` windows generous — timing is wall-clock on a live link.

## Next: start SIL-first

Develop suites against the virtual network first (deterministic, no hardware),
then — like CAN — run them against host-compiled firmware (`udp-sil` nodes) and
finally on a real link, switching only the interface.
