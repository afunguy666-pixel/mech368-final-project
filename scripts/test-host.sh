#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
for test in tests/*_test.cpp; do
  executable="$build_dir/$(basename "${test%.cpp}")"
  "${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic \
    -I libraries/Mech368Safety/src -I libraries/Mech368BLE/src \
    "$test" -o "$executable"
  "$executable"
  printf '%s\n' "通过：$test"
done
