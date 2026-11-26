#!/usr/bin/env bash
# Build the firmware-in-the-loop simulator around the unmodified sketches and
# run its scenarios. Each scenario runs in a fresh process (sketch globals are
# initialised once per "power-up", as on the board).
#
#   bash scripts/run-fw-sim.sh                      # all scenarios, summary only
#   bash scripts/run-fw-sim.sh estop_latch          # one scenario, full log
#   bash scripts/run-fw-sim.sh -v link_boot line_follow
#   bash scripts/run-fw-sim.sh --list
#   bash scripts/run-fw-sim.sh --seeds 20 link_stability   # Monte-Carlo: seeds 1..20
#   FWSIM_SKETCHES=~/try/remote_v2 bash scripts/run-fw-sim.sh --sketch remote=remote_v2
#                                                   # run scenarios against a modified copy
#
# Sketches compiled in: every firmware/<name>/<name>.ino and
# sim/examples/<name>/<name>.ino, plus directories listed in FWSIM_SKETCHES.
# Reports (CSV, log, HTML replay) go to build/fwsim/ (override with --out DIR).
set -euo pipefail
cd "$(dirname "$0")/.."

out=build/fwsim
verbose=auto
list=0
seeds=0
passthrough=()
scenarios=()
while [ $# -gt 0 ]; do
  case "$1" in
    --out) out="$2"; shift 2 ;;
    -v|--verbose) verbose=1; shift ;;
    -q|--quiet) verbose=0; shift ;;
    --list) list=1; shift ;;
    --seeds) seeds="$2"; shift 2 ;;
    --sketch) passthrough+=(--sketch "$2"); shift 2 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) scenarios+=("$1"); shift ;;
  esac
done

cxx="${CXX:-c++}"
build=build/fwsim-build
gen="$build/gen"   # three levels below the repository root (see ../../../ below)
rm -rf "$gen"
mkdir -p "$gen" "$build/obj" "$out"

{ printf 'R"FWSIMHTML('; cat sim/fwsim/viewer.html; printf ')FWSIMHTML"\n'; } > "$gen/viewer_html.inc"

includes=(-I sim/fwsim/hostduino -I sim/fwsim -I "$gen")
for lib in libraries/*/src; do includes+=(-I "$lib"); done
std=(-std=c++17 -O1 -g)
# Extra compiler/linker flags, e.g. CXXFLAGS="-fsanitize=address,undefined".
read -r -a user_flags <<< "${CXXFLAGS:-}"
std+=(${user_flags[@]+"${user_flags[@]}"})

sketch_dirs=(firmware/*/ sim/examples/*/)
# shellcheck disable=SC2206
[ -n "${FWSIM_SKETCHES:-}" ] && sketch_dirs+=(${FWSIM_SKETCHES})
sources=()
for dir in "${sketch_dirs[@]}"; do
  dir="${dir%/}"
  name="$(basename "$dir")"
  [ -f "$dir/$name.ino" ] || continue
  ns="sketch_$(printf '%s' "$name" | tr -c 'A-Za-z0-9_' '_')"
  case "$dir" in /*) rel="$dir" ;; *) rel="../../../$dir" ;; esac
  wrapper="$gen/$ns.cpp"
  {
    echo "// Generated from $dir by scripts/run-fw-sim.sh. Do not edit."
    echo '#include <Arduino.h>'
    # Library headers go at global scope; the sketch body is namespaced.
    grep -hE '^[[:space:]]*#[[:space:]]*include[[:space:]]*<' "$dir"/*.ino "$dir"/*.h 2>/dev/null | sort -u || true
    echo '#include "Runtime.h"'
    echo "namespace $ns {"
    echo "#include \"$rel/$name.ino\""
    for extra in "$dir"/*.ino; do
      [ "$extra" = "$dir/$name.ino" ] || echo "#include \"$rel/$(basename "$extra")\""
    done
    echo "}  // namespace $ns"
    echo "static fwsim::SketchRegistrar fwsim_sketch_registrar(\"$name\", &$ns::setup, &$ns::loop);"
  } > "$wrapper"
  sources+=("$wrapper:$dir")
done

objects=()
compile() {  # source, object, extra flags...
  local src="$1" obj="$2"; shift 2
  "$cxx" "${std[@]}" "${includes[@]}" "$@" -c "$src" -o "$obj"
}
for src in sim/fwsim/*.cpp sim/fwsim/scenarios/*.cpp libraries/*/src/*.cpp; do
  obj="$build/obj/$(basename "${src%.cpp}").o"
  compile "$src" "$obj" -Wall -Wextra -Werror -pedantic
  objects+=("$obj")
done
for entry in "${sources[@]}"; do
  src="${entry%%:*}"; dir="${entry#*:}"
  obj="$build/obj/$(basename "${src%.cpp}").o"
  # Sketch code: warnings shown but not fatal (the board build is the authority).
  compile "$src" "$obj" -Wall -I "$dir"
  objects+=("$obj")
done
"$cxx" "${std[@]}" "${objects[@]}" -o "$build/fwsim"

if [ "$list" = 1 ]; then
  "$build/fwsim" --list
  exit 0
fi
if [ ${#scenarios[@]} -eq 0 ]; then
  while IFS=$'\t' read -r name _; do scenarios+=("$name"); done < <("$build/fwsim" --list)
  [ "$verbose" = auto ] && verbose=0
fi
[ "$verbose" = auto ] && verbose=1

if [ "$seeds" -gt 0 ]; then
  failed=0
  for scenario in "${scenarios[@]}"; do
    passed=0
    failing=()
    for seed in $(seq 1 "$seeds"); do
      if "$build/fwsim" --quiet --seed "$seed" ${passthrough[@]+"${passthrough[@]}"} --out "$out" "$scenario" > "$out/$scenario-seed$seed.txt"; then
        passed=$((passed + 1))
      else
        failing+=("$seed")
      fi
    done
    echo "$scenario: $passed/$seeds seeds passed${failing[*]:+ (failing seeds: ${failing[*]})}"
    [ "$passed" -eq "$seeds" ] || failed=$((failed + 1))
  done
  echo "Reports: $out/<scenario>-seed<N>.html"
  [ "$failed" -eq 0 ]
  exit
fi

failed=0
for scenario in "${scenarios[@]}"; do
  if [ "$verbose" = 1 ]; then
    "$build/fwsim" ${passthrough[@]+"${passthrough[@]}"} --out "$out" "$scenario" || failed=$((failed + 1))
  else
    "$build/fwsim" --quiet ${passthrough[@]+"${passthrough[@]}"} --out "$out" "$scenario" || failed=$((failed + 1))
  fi
done
echo "${#scenarios[@]} scenario(s), $failed failed. Open $out/<scenario>.html in a browser to replay."
[ "$failed" -eq 0 ]
