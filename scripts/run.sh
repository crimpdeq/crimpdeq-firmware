#!/bin/sh
# Cargo runner. `probe-rs run` monitors the target serial port in newer probe-rs
# versions, so with defmt logging off it only flashes and resets the target. The
# download flag that resets differs between probe-rs versions, so reset separately.
set -eu

flash_args="--chip esp32c3 --idf-partition-table partitions.csv --preverify --restore-unwritten"

if [ "$DEFMT_LOG" = "off" ]; then
  probe-rs download $flash_args "$@"
  exec probe-rs reset --chip esp32c3
fi

exec probe-rs run $flash_args --no-location --always-print-stacktrace "$@"
