#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic \
  -I libraries/Mech368Safety/src tests/motion_safety_test.cpp \
  -o "$build_dir/motion_safety_test"
"$build_dir/motion_safety_test"
printf '%s\n' 'Host motion safety tests passed.'
