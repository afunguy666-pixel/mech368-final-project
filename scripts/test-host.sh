#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
includes=()
for lib in libraries/*/src; do includes+=(-I "$lib"); done
for test in tests/*_test.cpp; do
  executable="$build_dir/$(basename "${test%.cpp}")"
  "${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic \
    "${includes[@]}" "$test" -o "$executable"
  "$executable"
  printf '%s\n' "PASS: $test"
done
