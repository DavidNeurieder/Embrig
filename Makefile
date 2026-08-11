# Embrig developer tasks. CI mirrors these targets so local and CI cannot drift.
#
#   make check     fmt + clippy (all features, warnings denied)
#   make test      workspace unit/integration tests (both feature sets)
#   make examples  run the SIL and UDP examples (no hardware needed)
#   make sil       run both SIL examples
#   make hil       bring up vcan0 and run the HIL example + CLI loopback (root)
#   make vcan-up   create and bring up vcan0 (root)
#   make vcan-down tear down vcan0 (root)
#   make ci        everything CI runs

.PHONY: check test examples sil hil vcan-up vcan-down ci

CARGO ?= cargo

check:
	$(CARGO) fmt --all --check
	$(CARGO) clippy --workspace --all-targets --features socketcan -- -D warnings

test:
	$(CARGO) test --workspace
	$(CARGO) test --workspace --features socketcan

examples: sil
	$(CARGO) run -q --example udp_rover --package embrig-test

sil:
	$(CARGO) run -q --example sil_firmware --package embrig-sil
	$(CARGO) run -q --example robot_sil --package embrig-sil

hil: vcan-up
	$(CARGO) run -q --example can_hil --package embrig-test --features socketcan
	$(CARGO) run -q --features socketcan --bin embrig -- test examples/ev-powertrain/vehicle.yaml scripts/loopback.yaml --interface vcan0 --check

vcan-up:
	./scripts/vcan-up.sh

vcan-down:
	./scripts/vcan-down.sh

ci: check test examples
