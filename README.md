# voxelize

Voxelize LAS/LAZ point clouds into a downsampled output where each point is a voxel center.

Published on crates.io as [`voxelaz`](https://crates.io/crates/voxelaz).

## CLI

```text
voxelize --input <path> --output <path> --voxel-size <meters> [--counts-as-intensity] [--include-classification <codes>]
```

- `--input` — input LAS/LAZ path (required).
- `--output` — output LAS/LAZ path (required).
- `--voxel-size` — edge length of each voxel in meters (required).
- `--counts-as-intensity` — store per-voxel input point counts and write them to LAS intensity (default: presence-only; intensity stays 0).
- `--include-classification` — comma-separated classification codes to keep (default: all).

## Library

```rust
use voxelaz::{ClassificationFilter, VoxelizeConfig, VoxelizeError};

let config = VoxelizeConfig {
    voxel_size: 0.5,
    classification_filter: ClassificationFilter::all(),
    counts_as_intensity: false,
};
match voxelaz::voxelize(input, output, &config) {
    Ok(()) => {}
    Err(VoxelizeError::InvalidVoxelSize) => { /* ... */ }
    Err(e) => return Err(e.into()),
}
```

Fallible APIs return `voxelaz::Result<T>` (`Result<T, VoxelizeError>`). LAS I/O failures surface as `VoxelizeError::Las`. The CLI binary still reports errors via `anyhow`; library callers can use `?` with `anyhow::Result` because `VoxelizeError` implements `std::error::Error`.

`VoxelizeConfig::new(voxel_size)` sets `classification_filter` to all codes and `counts_as_intensity` to `false`.

## Live scene

`voxelaz::live` is a separate, persistent scene for a stream of world-frame points or sensor-to-hit rays. It does not read LAS/LAZ files and it does not depend on the offline grid. One `PointSceneWriter` or `RaySceneWriter` owns the mutable chunks. `insert_point`, `insert_points`, `insert_ray`, and `insert_rays` take `&mut self`. Any number of `SceneReader` handles can be cloned onto other threads. Readers see the last successful `publish`, not each inserted observation.

```text
use voxelaz::live::{open_presence_scene, PointSceneConfig, VoxelGeometry};

let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.2)?;
let (mut writer, reader) = open_presence_scene(geometry, &PointSceneConfig::default())?;
writer.insert_points(&[[0.1, 0.2, 0.3]])?;
assert!(reader.get_voxel_world([0.1, 0.2, 0.3])?.is_none());
writer.publish()?;
assert!(reader.get_voxel_world([0.1, 0.2, 0.3])?.is_some());
```

Count mode is `open_count_scene`. Ray mode is `open_ray_scene` with a `RayUpdateModel`. `VoteModel` stores `Votes { hits, misses }` per voxel in 8 bytes: one miss for each voxel a ray crosses and one hit for the voxel it ends in.

- **Units:** counts are kept in 1/256 of a vote (`Votes::UNIT`). `Votes::from_votes`, `hit_votes` and `miss_votes` convert. `Votes::hit_share` is hits as a fraction of all votes.
- **Decay:** `VoteModel::new(VoteDecay)` selects it; `VoteModel::default()` is `VoteDecay::None`.
  - `Exponential { keep }` multiplies a voxel's counts by `keep`.
  - `CapHalving { cap_votes }` halves them once their total passes the cap.
  - Both happen at each `publish`, to voxels updated since the previous publish. Voxels that were not observed are never decayed.
- **Order and overflow:** the map does not depend on ray order between two publishes. A counter that reaches 2^30 units halves both counts, keeping their ratio.
- **Blocked rule:** `BlockedRule::new(min_hits, min_hit_share)` decides when a reader treats a voxel as blocked; `min_hits` is in votes and may be fractional. `BlockedRule::any_hit()` blocks on any hit. `is_blocked` uses only an integer shift, multiply and compare. The share is rounded down to 1/65536, which can only block more voxels, never fewer. The writer never applies the rule. `LogOddsOccupancy` is a reference clamp, `new = clamp(old + hit_or_free_delta, min, max)`, and is not a sensor model. Unknown cells are `None`. A stored `0.0` in ray mode is an observed value. `log_odds_to_probability` is for use outside ingestion.

`PairedSceneWriter` (prototype) keeps two aligned maps fed by the same rays: a coarse `VoteModel` ray map at the given voxel size, and a fine hit-only map at half that size.

- **Alignment:** the coarse map uses chunk side 16 and the fine map 32, so both have the same chunk keys and each coarse voxel holds exactly 8 fine voxels.
- **Publishing:** `publish` installs the coarse votes, the fine hits and an 8-bit sub-voxel mask per coarse voxel, all under one generation. The mask sets bit x + 2y + 4z for each fine sub-voxel that has a hit. Read them through `PairedReader::snapshot_region` or `snapshot_all`.
- **Clearing:** with a decaying vote model and `ClearRule::new(max_hit_votes, min_miss_votes)`, a fine voxel is cleared at publish when its coarse parent was observed in that pass and, after decay, has at most `max_hit_votes` hits and at least `min_miss_votes` misses. A fine voxel that shares a coarse voxel with a surface still being hit is never cleared this way.

`insert_scan_from_source` (prototype) takes the same input as `insert_rays_from_source` and gives the same map and statistics, but writes each voxel once per call. It needs a `CountedRayModel`, a model whose result depends only on how many hit and free events a cell gets; `VoteModel` is one.

- **Counting:** each ray's events are counted in a scan-local accumulator, in blocks aligned with the map's chunks and found through a table over the scan's box. Then each touched voxel gets its hit and free counts in one `apply_counts`. A scan makes about 19 events per distinct voxel, most of them near the sensor.
- **Threads:** `insert_scan_from_source_threads(source, hits, n)` counts `n` contiguous slices of the rays on `n` threads and adds them to the map in order on the calling thread. The map keeps one writer and does not depend on `n`.
- **Limits:** with `max_chunks` set it falls back to the per-ray path. Calls are split into batches of `SCAN_BATCH_RAYS` (2^20) rays. A vote count that crosses 2^30 units inside one batch is halved at its next update instead of part way through.

The grid is infinite and addressed with signed voxels. Chunks are 16³ cells by default (`PointSceneWriter<Cell, SIDE>` can be built at 8 or 32). Sparse chunks promote to a dense array at `promotion_threshold` (default 256). `div_euclid` maps negative voxels into chunks. Coordinates that are non-finite or not exactly representable as a voxel index are counted invalid and are not saturated to an extreme key.

`RangeFilter` compares squared sensor-to-hit distance, inclusive, with no square root. Point mode applies it in `insert_point_from_source`, `insert_points_from_source`, and `insert_point_observations`. `insert_point` and `insert_points` have no sensor origin: when a filter is configured they return `LiveError::RangeFilterRequiresSource` and write nothing. A rejected hit rejects the whole ray and writes no voxels. `max_chunks` and `max_cells_per_ray` reject an observation before mutating it. `None` for the ray cap still stops at `ABSOLUTE_MAX_CELLS_PER_RAY` so a ray cannot run forever. The default cap is `DEFAULT_MAX_CELLS_PER_RAY` (1_048_576). The bound is the Manhattan voxel distance plus one, so a configured cap can reject a shorter ray.

`publish` copies only dirty chunks into `Arc` snapshots, then updates the reader index under one write lock. `snapshot_chunks` looks up the requested keys. `snapshot_region` looks up chunk positions inside the bounds when that box is no larger than the published map, and scans the map otherwise. `snapshot_all` is proportional to the number of chunks. Each of these copies chunk handles under one read lock and records one generation. A snapshot stays immutable after later publishes. `snapshot_changes_since` reads the journal and the changed chunk handles under that same lock; store `snapshot.generation()` as the cursor. A later `snapshot_chunks` can carry a newer generation than an earlier `changes_since`, and advancing the cursor to that newer generation skips chunk keys from the publish in between. A cursor older than the bounded journal returns `ChangesError::HistoryLost`; take `snapshot_all` and continue from that snapshot's generation. A cursor past the current generation returns `ChangesError::CursorAhead`. The journal keeps at least the newest publication even when that publication exceeds the key budget. An empty `publish` does not advance the generation.

The writer map and the latest published chunks can both be resident, and a slow reader can keep older `Arc` chunks alive. `memory_stats` estimates those costs. There is no automatic eviction. Readers never call the ray model. Ray updates run in input order. DDA emits one `Free` event for each positive-length interior cell and exactly one `Hit` on the endpoint voxel, including a zero-length endpoint. Corner and edge touches are not marked free. `ray_length_m` is computed only when `needs_metric_length` returns true.

Examples: `live_points`, `live_rays`, `live_observer`. `bench_scan` replays synthetic LiDAR scans through the writer, and `scripts/bench_board.sh` runs its matrix on the target board. Host measurements are in `docs/live-benchmarks.md`. ROCK 5T numbers require a measurement on that board.
