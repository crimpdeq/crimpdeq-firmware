#!/bin/sh
# Cargo runner. `probe-rs run` always monitors the target serial port, so with
# defmt logging off it only flashes and starts the firmware.
set -eu

flash_args="--chip esp32c3 --idf-partition-table partitions.csv --preverify --restore-unwritten"

if [ "$DEFMT_LOG" = "off" ]; then
  exec probe-rs download $flash_args --start "$@"
fi

exec probe-rs run $flash_args --no-location --always-print-stacktrace "$@"
