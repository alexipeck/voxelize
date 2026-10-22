# Live scene measurements

Two harnesses:

- `examples/bench_scan` replays synthetic LiDAR scans of a warehouse corridor (floor, walls, ceiling, pillars, crates) through the writer on its own thread, fed by a producer over a bounded channel. It reports input rate, cells per ray, cell events/s, per-scan ingest time, whether a paced writer keeps up, publish percentiles, per-thread CPU, RSS and A*-style reader lookups. This is the one to quote.
- `examples/bench_live` runs repeated micro cases (one voxel, one ray) in a tight loop. These are best cases, useful for regressions only.

## Board runs

On the ROCK 5T, from the repository root:

```bash
scripts/bench_board.sh
```

`--quick` checks the setup in a few minutes. The script pins to the highest-capacity cores (the Cortex-A76 cluster), records the board, kernel, compiler, governors, clocks, temperatures and load in `system.txt`, samples temperatures and clocks every 2 s into `thermal.csv`, and writes one row per case to `summary.md` under `bench-results/<time>-<host>/`. It builds a generic binary and one with `-C target-cpu=cortex-a76`.

No ROCK 5T results yet. Host numbers below are not evidence for the board.

## Host setup

| Item | Value |
| --- | --- |
| Machine | Apple M4, 10 cores, 32 GB |
| OS | macOS 26.6.2 (25G83) |
| Compiler | rustc 1.99.0 (`b940084d7`, 2026-09-28), host `aarch64-apple-darwin`, LLVM 23.1.1 |
| Code | `858a388` plus the 2026-10-21 working-tree changes (benchmarks, lint fixes, a ray error-path cleanup; no hot-path change) |
| Profile | `--release`, no `target-cpu`, no LTO |

macOS has no `/proc`, so CPU, writer busy and RSS are `n/a` here. A paced run stops at its wall-clock length even if the writer has fallen behind, and reports the backlog as late scans and lag.

## Realistic scans (`bench_scan`, 8 s per case)

Sensor: 128 channels over ±22.5°, 10 Hz, about 230,000 returns per scan (2.3 M/s), 40 m range, ±2 cm range noise. The sensor flies along the corridor at 4 m/s unless noted. Voxels 0.2 m, chunk side 16, reference `LogOddsOccupancy`, publish at 10 Hz.

| case | mode | voxel m | paced | input M/s | cells/ray | M cell events/s | publish p99 ms | late scans | observer M lookups/s |
|---|---|---|---|---|---|---|---|---|---|
| presence-max | presence | 0.2 | no | 53.8 | - | 53.8 | 0.44 | 0 | - |
| count-max | count | 0.2 | no | 47.3 | - | 47.3 | 1.03 | 0 | - |
| rays-max | rays | 0.2 | no | 0.85 | 89 | 77.4 | 1.65 | 0 | - |
| rays-moving-max | rays, moving source | 0.2 | no | 0.86 | 89 | 78.6 | 1.84 | 0 | - |
| rays-evidence-max | rays, two-counter cell | 0.2 | no | 0.82 | 89 | 74.7 | 1.63 | 0 | - |
| presence-paced | presence | 0.2 | 2.3 M/s | 2.33 | - | 2.3 | 0.72 | 0 | - |
| rays-paced | rays | 0.2 | 2.3 M/s | 0.89 | 89 | 81.0 | 1.13 | 31 of 32 (4.9 s behind at the end) | - |
| rays-paced-500k | rays | 0.2 | 0.5 M/s | 0.50 | 89 | 45.9 | 1.30 | 0 | - |
| rays-paced-observer | rays + region snapshot reader | 0.2 | 0.5 M/s | 0.50 | 89 | 45.9 | 1.34 | 0 | 49.7 |
| rays-max-observer-locked | rays + locked reader | 0.2 | no | 0.86 | 89 | 78.3 | 1.62 | 0 | 65.4 |
| rays-max-voxel-0.1 | rays | 0.1 | no | 0.42 | 178 | 77.1 | 11.56 | 0 | - |
| rays-max-voxel-0.4 | rays | 0.4 | no | 1.66 | 45 | 75.5 | 0.27 | 0 | - |
| rays-max-range-15 | rays | 0.2 | no | 1.55 | 47 | 76.4 | 0.25 | 0 | - |
| rays-max-side-8 | rays | 0.2 | no | 0.70 | 89 | 63.5 | 2.74 | 0 | - |
| rays-max-side-32 | rays | 0.2 | no | 0.92 | 89 | 83.5 | 2.90 | 0 | - |
| rays-max-hover | rays, hovering | 0.2 | no | 0.85 | 89 | 76.9 | 6.08 | 0 | - |

What this shows on the host:

- Point modes clear 2.3 M/s by a wide margin. Paced at 2.3 M/s, each 230k-point scan takes a few milliseconds.
- Ray cost follows cells per ray. A single writer does about 75–83 M cell events/s regardless of voxel size or range, so the ray rate is about that figure divided by cells per ray. At 0.2 m and 40 m that is 0.85 M rays/s. In the paced 2.3 M/s run the writer processed 32 of the 80 scans in 8 s, and every scan after the first was late.
- About 80% of each cell event is the store and model update, not DDA stepping. On the same scans the bare traversal runs at about 3.4 M rays/s (about 310 M cell steps/s). The sparse-to-dense threshold (1, 64 or 256) made no difference for rays.
- Publishing at 10 Hz costs about 1–2 ms (p99) at 0.2 m, rising to about 6 ms when hovering and 12 ms at 0.1 m, where each publish copies more dense chunks.
- A reader walking 26-neighbourhoods gets 50–65 M lookups/s on one core, through a region snapshot or the locked reader.

## 0.1 m voxels (`bench_scan`, 20 s, max rate)

| chunk side | cell | cells/ray (mean / p95) | M rays/s | M cell events/s | publish p99 ms | est. MB per copy (working, published) |
|---|---|---|---|---|---|---|
| 16 | f32 log-odds | 178 / 395 | 0.44 | 79.3 | 5.7 | 87 |
| 32 | f32 log-odds | 178 / 395 | 0.49 | 89.3 | 6.3 | 135 |
| 16 | two u16 counters | 178 / 395 | 0.43 | 77.9 | 5.7 | 86 |
| 32 | two u16 counters | 178 / 395 | 0.49 | 88.1 | 6.6 | 135 |

At 0.1 m one host writer is 4.7–5.3× short of 2.3 M rays/s. Paced at 2.3 M/s for 3 s, it processed 6 scans, 5 of them late.

## Voxel size with votes (`bench_scan --model votes`, 8 s, max rate)

| voxel | sensor | cells/ray (mean / p95) | M rays/s | dirty chunks per publish | publish p50 / p99 ms | est. MB per copy |
|---|---|---|---|---|---|---|
| 0.2 m | band ±22.5° | 89 / 197 | 0.87 | 785 | 0.94 / 2.29 | 26 |
| 0.2 m | upper hemisphere | 75 / 166 | 1.02 | 608 | 0.87 / 2.40 | 25 |
| 0.25 m | band ±22.5° | 72 / 158 | 1.07 | 535 | 0.53 / 1.06 | 15 |
| 0.25 m | upper hemisphere | 60 / 133 | 1.25 | 428 | 0.48 / 1.12 | 14 |
| 0.4 m | band ±22.5° | 45 / 99 | 1.62 | 208 | 0.20 / 0.44 | 6 |
| 0.4 m | upper hemisphere | 38 / 83 | 1.88 | 142 | 0.17 / 0.40 | 6 |

Point presence at 0.25 m: 50 M points/s.

## Field of view and pass rate (0.1 m, `bench_scan`, 10 s, max rate)

Two u32 votes per voxel (`--model votes`), except where noted.

| sensor | pass rate | cells/ray (mean / p95) | M rays/s | dirty chunks per publish | publish p50 / p99 ms |
|---|---|---|---|---|---|
| band ±22.5° | 10 Hz | 178 / 395 | 0.41 | 4,600 | 7.2 / 38.8 |
| upper hemisphere 0–90° | 10 Hz | 150 / 332 | 0.49 | 4,106 | 7.5 / 28.3 |
| upper hemisphere 0–90° | 20 Hz | 150 / 332 | 0.49 | 4,059 | 6.3 / 9.6 |
| upper hemisphere 0–90° | 40 Hz | 150 / 332 | 0.49 | 3,959 | 6.1 / 8.3 |
| upper hemisphere, f32 log-odds | 10 / 20 / 40 Hz | 150 / 332 | 0.49 | about 4,000 | 4.1 / 4.0 / 3.8 (p50) |
| lower hemisphere −90–0° | 10 Hz | 58 / 174 | 1.24 | 1,253 | 1.3 / 4.0 |

Every pass dirties nearly every chunk in view, whatever its point count, so publish cost per second grows with pass rate. The p99 outliers in the first two rows (39 ms and 28 ms) were not reproduced in the other runs and are worth checking on the board. Images of the scene are in `docs/images/test-scene/` (`cargo run --release --example scene_render`).

## Micro cases (`bench_live`)

Re-run at `858a388`. Earlier copies of this file named `9b3c2c3` plus uncommitted work; the rates below match those within run-to-run noise.

| Workload | Rate |
| --- | --- |
| Point presence, same voxel | 119 million points/s |
| Point presence, clustered chunks | 100 million points/s |
| Point presence, expanding | 25.4 million points/s |
| Point count, same voxel | 139 million points/s |
| Point presence, chunk side 8 / 16 / 32 | 144 / 132 / 142 million points/s |
| Point presence, promotion 64 / 128 / 256 / 512 | 115 / 115 / 78 / 89 million points/s |
| Ray, one repeated axis-aligned ray, 5 / 10 / 20 / 40 / 80 cells | 83 / 81 / 68 / 71 / 77 million cell events/s |
| Ray, 10 cells, publish every batch | 8.14 million rays/s |
| Point filter, 100% / 50% / 10% kept | 118 / 192 / 379 million inputs/s |
| Publish 2000 new sparse chunks | 0.208 ms |
| Point ingest while another thread loops `snapshot_all` on a one-chunk scene | 129 million points/s |

The repeated-ray rows reuse one cache-resident ray. Their cell-event rates are close to the realistic ones, but their ray rates come from 5–80-cell rays, so they do not describe a 0.2 m, 40 m sensor. Quote `bench_scan` instead.
