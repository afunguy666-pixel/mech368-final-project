#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
for sketch in firmware/robot firmware/remote; do
  arduino-cli compile --fqbn arduino:mbed_nano:nano33ble \
    --libraries libraries "$sketch"
done
