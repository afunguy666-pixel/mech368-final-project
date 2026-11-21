#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
output_dir="${1:-build/sil}"
bash scripts/test-host.sh
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic \
  -I libraries/Mech368Safety/src -I libraries/Mech368BLE/src \
  sim/main.cpp -o "$build_dir/sil_sim"
mkdir -p -- "$output_dir"
"$build_dir/sil_sim" "$output_dir"
