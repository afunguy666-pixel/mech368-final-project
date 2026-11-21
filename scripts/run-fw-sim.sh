#!/usr/bin/env bash
# 用未经修改的程序构建固件在环模拟器，并运行测试场景。每个场景都在新的进程中运行
# （和开发板一样，程序的全局变量每次"上电"只初始化一次）。
#
#   bash scripts/run-fw-sim.sh                      # 全部场景，只显示汇总
#   bash scripts/run-fw-sim.sh estop_latch          # 单个场景，显示完整日志
#   bash scripts/run-fw-sim.sh -v link_boot line_follow
#   bash scripts/run-fw-sim.sh --list               # 列出所有场景
#   bash scripts/run-fw-sim.sh --seeds 20 link_stability   # 蒙特卡洛：种子 1..20
#   FWSIM_SKETCHES=~/try/remote_v2 bash scripts/run-fw-sim.sh --sketch remote=remote_v2
#                                                   # 用修改过的副本运行场景
#
# 会编译进来的程序：所有 firmware/<名字>/<名字>.ino 和
# sim/examples/<名字>/<名字>.ino，以及 FWSIM_SKETCHES 中列出的目录。
# 报告（CSV、日志、HTML 回放）输出到 build/fwsim/（可用 --out 目录 修改）。
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
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) scenarios+=("$1"); shift ;;
  esac
done

cxx="${CXX:-c++}"
build=build/fwsim-build
gen="$build/gen"   # 位于仓库根目录下三层（见下面的 ../../../）
rm -rf "$gen"
mkdir -p "$gen" "$build/obj" "$out"

{ printf 'R"FWSIMHTML('; cat sim/fwsim/viewer.html; printf ')FWSIMHTML"\n'; } > "$gen/viewer_html.inc"

includes=(-I sim/fwsim/hostduino -I sim/fwsim -I "$gen")
for lib in libraries/*/src; do includes+=(-I "$lib"); done
std=(-std=c++17 -O1 -g)
# 额外的编译/链接参数，例如 CXXFLAGS="-fsanitize=address,undefined"。
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
    echo "// 由 scripts/run-fw-sim.sh 根据 $dir 生成，请勿编辑。"
    echo '#include <Arduino.h>'
    # 库头文件放在全局作用域；程序本体放进独立的命名空间。
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
for src in sim/fwsim/*.cpp sim/fwsim/scenarios/*.cpp; do
  obj="$build/obj/$(basename "${src%.cpp}").o"
  compile "$src" "$obj" -Wall -Wextra -Werror -pedantic
  objects+=("$obj")
done
for entry in "${sources[@]}"; do
  src="${entry%%:*}"; dir="${entry#*:}"
  obj="$build/obj/$(basename "${src%.cpp}").o"
  # 程序代码：显示警告但不视为错误（以开发板构建结果为准）。
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
    echo "$scenario：$passed/$seeds 个种子通过${failing[*]:+（失败的种子：${failing[*]}）}"
    [ "$passed" -eq "$seeds" ] || failed=$((failed + 1))
  done
  echo "报告：$out/<场景>-seed<N>.html"
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
echo "共 ${#scenarios[@]} 个场景，$failed 个失败。用浏览器打开 $out/<场景>.html 查看回放。"
[ "$failed" -eq 0 ]
