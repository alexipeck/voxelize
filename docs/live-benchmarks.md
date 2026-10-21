# Live scene host measurements

Measured on 2026-10-21 from commit `9b3c2c3` plus the uncommitted `voxelaz::live` work. Two consecutive runs of `cargo run --release --example bench_live` produced rates in the same band (the largest gap was the expanding point case, 26.5 million/s then 28.3 million/s). The table is the second run.

## Setup

| Item | Value |
| --- | --- |
| Machine | Apple M4, 10 cores, 32 GB |
| OS | macOS 26.6.2 (25G83) |
| Compiler | rustc 1.99.0 (`b940084d7`, 2026-09-28), host `aarch64-apple-darwin`, LLVM 23.1.1 |
| Profile | `cargo run --release --example bench_live` (default release, no `target-cpu`, no LTO) |
| Window | one untimed warmup, then about 250–600 ms of repetitions |
| Inputs | synthetic and repeated; the harness is `examples/bench_live.rs` |

This run does not sample per-core CPU, RSS, or a multi-minute thermal soak. It does not include a LiDAR driver or a registration pipeline. ROCK 5T figures require a run on that board, with its clocks, governor, and the CPU budget shared with the rest of the vehicle.

## Results

| Workload | Second run |
| --- | --- |
| Point presence, same voxel | 118 million points/s |
| Point presence, clustered chunks | 89.0 million points/s |
| Point presence, expanding | 28.3 million points/s |
| Point count, same voxel | 140 million points/s |
| Point presence, chunk side 8 | 128 million points/s |
| Point presence, chunk side 16 | 129 million points/s |
| Point presence, chunk side 32 | 143 million points/s |
| Point presence, promotion 64 | 115 million points/s |
| Point presence, promotion 128 | 115 million points/s |
| Point presence, promotion 256 | 79.2 million points/s |
| Point presence, promotion 512 | 79.2 million points/s |
| Ray, 5 cells, no publish in the loop | 15.2 million rays/s, 75.9 million cell events/s |
| Ray, 10 cells | 7.87 million rays/s, 78.7 million cell events/s |
| Ray, 20 cells | 3.37 million rays/s, 67.4 million cell events/s |
| Ray, 40 cells | 1.75 million rays/s, 70.2 million cell events/s |
| Ray, 80 cells | 0.942 million rays/s, 75.4 million cell events/s |
| Ray, 10 cells, publish every batch | 7.84 million rays/s, 78.4 million cell events/s |
| Point filter, all kept | 115 million inputs/s |
| Point filter, half kept | 186 million inputs/s |
| Point filter, 10% kept | 376 million inputs/s |
| Publish 2000 new sparse chunks | 0.201 ms; the following empty publish stayed on generation 1 |
| Point ingest while another thread calls `snapshot_all` | 128 million points/s, 10.4 million snapshots |

Ray rows use voxel size 1 m and the reference `LogOddsOccupancy` model. Point rows use voxel size 0.2 m. The same-voxel and clustered cases hit chunks that already exist. The expanding case allocates new chunks during the timed pass. Chunk-side rows repeat one voxel, so they do not stress chunk boundaries. The promotion workload fills about 150 occupied cells in each of two chunks; thresholds 64 and 128 have already promoted to a dense bitset before the timed loop, and thresholds 256 and 512 are still sparse. The default threshold remains 256.

On this host, repeated point ingestion and repeated rays of 5, 10, and 20 cells are above 2.3 million accepted observations/s. Repeated rays of 40 and 80 cells are below that rate. Cell-event throughput stays near 67–79 million/s across those ray lengths, so longer rays cost proportionally more. Publishing every batch of 10-cell rays changed the ray rate from 7.87 million/s to 7.84 million/s in this repeated case. A spinning observer taking snapshots of a one-chunk scene left the point writer near its single-thread rate.

The 2.3 million observations/s ROCK 5T goal is unmet as a device claim. The dominating measured cost for rays on this host is the per-cell update, about 70 million cell events/s in this harness.
