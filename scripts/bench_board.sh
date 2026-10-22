#!/usr/bin/env bash
# Runs the live-scene benchmark matrix on the target board (Linux) and records the
# machine state next to the results.
#
#   scripts/bench_board.sh [--quick] [--seconds N] [--soak-seconds N] [--voxel M] [--cores LIST]
#
#   --quick          5 s per case and a 60 s soak, to check the setup
#   --seconds N      length of each case (default 30)
#   --soak-seconds N length of the sustained thermal run (default 600)
#   --voxel M        voxel size in metres for every case that does not set its own (default 0.2)
#   --cores LIST     CPUs to pin to, for taskset (default: the highest-capacity cores,
#                    which are the Cortex-A76 cores on an RK3588)
#
# Results go to bench-results/<UTC time>-<hostname>/:
#   system.txt   git commit, board, kernel, compiler, governors, clocks, temperatures, load
#   <case>.txt   full output of each case
#   summary.md   one table row per case
#   thermal.csv  temperatures and CPU clocks, sampled every 2 s for the whole run
#   parallel.txt combined rate of the 2- and 4-scene runs
#
# Needs a Rust toolchain on the board. Builds a generic binary and one tuned with
# -C target-cpu=cortex-a76 and one with fat LTO, and compares them.
set -euo pipefail

seconds=30
soak=600
voxel=0.2
cores=""
while (($#)); do
    case $1 in
    --quick) seconds=5 soak=60 ;;
    --seconds) seconds=$2; shift ;;
    --soak-seconds) soak=$2; shift ;;
    --voxel) voxel=$2; shift ;;
    --cores) cores=$2; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

[[ $(uname -s) == Linux ]] || { echo "bench_board.sh runs on the Linux target" >&2; exit 2; }
cd "$(git rev-parse --show-toplevel)"
out="bench-results/$(date -u +%Y%m%dT%H%M%SZ)-$(hostname)"
mkdir -p "$out"

big_cores() {
    local cpu cap max=0 list=()
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        [[ -r $cpu/cpu_capacity ]] || continue
        cap=$(<"$cpu/cpu_capacity")
        ((cap > max)) && max=$cap
    done
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        [[ -r $cpu/cpu_capacity && $(<"$cpu/cpu_capacity") == "$max" ]] && list+=("${cpu##*cpu}")
    done
    local IFS=,
    echo "${list[*]}"
}
little_cores() {
    local cpu cap min=0 list=()
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        [[ -r $cpu/cpu_capacity ]] || continue
        cap=$(<"$cpu/cpu_capacity")
        ((min == 0 || cap < min)) && min=$cap
    done
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        [[ -r $cpu/cpu_capacity && $(<"$cpu/cpu_capacity") == "$min" ]] && list+=("${cpu##*cpu}")
    done
    local IFS=,
    echo "${list[*]}"
}
[[ -n $cores ]] || cores=$(big_cores)
little=$(little_cores)
pin=()
if [[ -n $cores ]] && command -v taskset >/dev/null; then
    pin=(taskset -c "$cores")
fi

read_or_blank() { cat "$1" 2>/dev/null || true; }

{
    echo "== git"
    git rev-parse HEAD
    git status --short
    echo "== board"
    tr -d '\0' </proc/device-tree/model 2>/dev/null || true
    echo
    echo "== kernel"
    uname -a
    echo "== os"
    head -4 /etc/os-release 2>/dev/null || true
    echo "== rustc"
    rustc -Vv
    echo "== cpus"
    lscpu 2>/dev/null || cat /proc/cpuinfo
    echo "== pinned to cpus: ${cores:-none}"
    echo "== cpu governor cur_khz max_khz capacity"
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        echo "${cpu##*/} $(read_or_blank "$cpu/cpufreq/scaling_governor") $(read_or_blank "$cpu/cpufreq/scaling_cur_freq") $(read_or_blank "$cpu/cpufreq/cpuinfo_max_freq") $(read_or_blank "$cpu/cpu_capacity")"
    done
    echo "== thermal (millidegrees C)"
    for zone in /sys/class/thermal/thermal_zone*; do
        echo "$(read_or_blank "$zone/type") $(read_or_blank "$zone/temp")"
    done
    echo "== gpu and opencl"
    for card in /sys/class/drm/card[0-9]; do
        [[ -e $card/device/driver ]] && echo "${card##*/} driver $(basename "$(readlink -f "$card/device/driver")")"
    done
    ls /dev/mali* /dev/dri 2>/dev/null || true
    read_or_blank /sys/class/misc/mali0/device/gpuinfo
    ls /etc/OpenCL/vendors 2>/dev/null || echo "no OpenCL ICD files in /etc/OpenCL/vendors"
    if command -v clinfo >/dev/null; then clinfo -l; clinfo | grep -Ei 'platform name|device name|version|global memory|compute units|max clock' || true; else echo "clinfo not installed"; fi
    echo "== memory"
    free -m
    echo "== load and busiest processes"
    uptime
    ps -eo pcpu,pmem,comm --sort=-pcpu | head -15
} >"$out/system.txt" 2>&1

sample() {
    local zone cpu started
    started=$(date +%s)
    printf time_s
    for zone in /sys/class/thermal/thermal_zone*; do printf ',%s' "$(read_or_blank "$zone/type")"; done
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do printf ',%s_khz' "${cpu##*/}"; done
    echo
    while true; do
        printf '%s' $(($(date +%s) - started))
        for zone in /sys/class/thermal/thermal_zone*; do printf ',%s' "$(read_or_blank "$zone/temp")"; done
        for cpu in /sys/devices/system/cpu/cpu[0-9]*; do printf ',%s' "$(read_or_blank "$cpu/cpufreq/scaling_cur_freq")"; done
        echo
        sleep 2
    done
}
sample >"$out/thermal.csv" &
sampler=$!
trap 'kill "$sampler" 2>/dev/null || true' EXIT

echo "building"
cargo build --release --example bench_scan --example bench_live
CARGO_TARGET_DIR=target/cortex-a76 RUSTFLAGS="-C target-cpu=cortex-a76" \
    cargo build --release --example bench_scan
# Generic code from the library is compiled in the example's own crate; whole-program
# optimisation changed ray throughput by about 15% on the host, so compare it here too.
CARGO_TARGET_DIR=target/lto CARGO_PROFILE_RELEASE_LTO=fat CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1 \
    cargo build --release --example bench_scan
generic=target/release/examples/bench_scan
tuned=target/cortex-a76/release/examples/bench_scan
lto=target/lto/release/examples/bench_scan

"$generic" --table-header >"$out/summary.md"

run() {
    local name=$1 binary=$2
    shift 2
    local size=(--voxel "$voxel")
    [[ " $* " == *" --voxel "* ]] && size=()
    echo "-- $name"
    ${pin[@]+"${pin[@]}"} "$binary" --label "$name" --seconds "$seconds" "${size[@]}" "$@" | tee "$out/$name.txt"
    grep '^row: ' "$out/$name.txt" | sed 's/^row: //' >>"$out/summary.md"
    # Let clocks and temperatures settle between cases.
    sleep 5
}

# Like run, pinned to the given CPUs instead of the default set.
run_on() {
    local cpus=$1
    shift
    if ! command -v taskset >/dev/null; then
        echo "-- $1 skipped: needs taskset"
        return
    fi
    local saved=(${pin[@]+"${pin[@]}"})
    pin=(taskset -c "$cpus")
    run "$@"
    pin=(${saved[@]+"${saved[@]}"})
}

# N independent scenes at once, one per pinned core, each with its own producer on the
# same core. Their combined rate is an upper bound for shard writers on these cores,
# including the shared memory bandwidth.
run_parallel() {
    local count=$1 name=$2 binary=$3
    shift 3
    local cpus i pids=()
    IFS=, read -r -a cpus <<<"$cores"
    if ((${#cpus[@]} < count)) || ! command -v taskset >/dev/null; then
        echo "-- $name skipped: needs taskset and $count pinned cores"
        return
    fi
    local size=(--voxel "$voxel")
    [[ " $* " == *" --voxel "* ]] && size=()
    echo "-- $name ($count scenes on cpus ${cpus[*]:0:count})"
    for ((i = 0; i < count; i++)); do
        taskset -c "${cpus[i]}" "$binary" --label "$name-$i" --seconds "$seconds" "${size[@]}" "$@" >"$out/$name-$i.txt" &
        pids+=($!)
    done
    wait "${pids[@]}"
    for ((i = 0; i < count; i++)); do
        grep '^row: ' "$out/$name-$i.txt" | sed 's/^row: //' >>"$out/summary.md"
    done
    awk -F'|' '/^row: / { sum += $6 } END { printf "%s: %d scenes, %.2f M inputs/s together\n", name, n, sum }' \
        name="$name" n="$count" "$out/$name"-*.txt | tee -a "$out/parallel.txt"
    sleep 5
}

# Throughput as fast as the writer can go: --voxel size (default 0.2 m), 40 m range,
# 2.3 M returns/s sensor, ±22.5° band unless a case says hemisphere.
run presence-max "$generic" --mode presence
run count-max "$generic" --mode count
run rays-max "$generic" --mode rays
run rays-max-a76 "$tuned" --mode rays
run rays-moving-max "$generic" --mode rays-moving
run rays-evidence-max "$generic" --mode rays --model evidence

# Fed at the sensor's real 2.3 M/s schedule: writer busy is the CPU cost, late scans
# say whether one writer keeps up.
run presence-paced "$generic" --mode presence --paced
run rays-paced "$generic" --mode rays --paced
run rays-paced-hemisphere "$generic" --mode rays --model votes --paced --elevation-min 0 --elevation-max 90
run rays-paced-500k "$generic" --mode rays --paced --rate 500000
run rays-paced-observer "$generic" --mode rays --paced --rate 500000 --observer snapshot
run rays-max-observer-locked "$generic" --mode rays --observer locked

# Geometry and storage variants.
run rays-max-hemisphere-votes "$generic" --mode rays --model votes --elevation-min 0 --elevation-max 90
run rays-max-hemisphere-votes-0.25 "$generic" --mode rays --model votes --elevation-min 0 --elevation-max 90 --voxel 0.25
run rays-max-voxel-0.1 "$generic" --mode rays --voxel 0.1
run rays-max-voxel-0.4 "$generic" --mode rays --voxel 0.4
run rays-max-range-15 "$generic" --mode rays --max-range 15
run rays-max-side-8 "$generic" --mode rays --chunk-side 8
run rays-max-side-32 "$generic" --mode rays --chunk-side 32
run rays-max-hover "$generic" --mode rays --speed 0

# How the cores scale: one A55 core, then 2 and 4 independent scenes on the A76 cores.
hemisphere=(--mode rays --model votes --elevation-min 0 --elevation-max 90)
if [[ -n $little && $little != "$cores" ]]; then
    run_on "${little%%,*}" rays-max-hemisphere-a55 "$generic" "${hemisphere[@]}"
fi
run_parallel 2 rays-max-hemisphere-x2 "$generic" "${hemisphere[@]}"
run_parallel 4 rays-max-hemisphere-x4 "$generic" "${hemisphere[@]}"

# Whole-program optimisation, and the paired coarse-ray plus half-size hit maps.
run rays-max-hemisphere-lto "$lto" "${hemisphere[@]}"
paired=(--mode paired --decay exp:0.7 --clear 0.25:1 --elevation-min 0 --elevation-max 90)
run paired-max "$generic" "${paired[@]}"
run paired-max-lto "$lto" "${paired[@]}"
run paired-paced-lto "$lto" "${paired[@]}" --paced

# Sustained run for thermal throttling; compare its rate with rays-max.
per_case=$seconds
seconds=$soak
run rays-soak "$generic" --mode rays
seconds=$per_case

echo "-- micro benchmarks"
${pin[@]+"${pin[@]}"} target/release/examples/bench_live | tee "$out/bench_live.txt"

if grep -q 'writer=n/a' "$out"/*.txt; then
    echo "warning: some cases could not read /proc CPU times; check $out/*.txt" >&2
fi
echo
cat "$out/summary.md"
[[ -f $out/parallel.txt ]] && cat "$out/parallel.txt"
echo
echo "results: $out"
