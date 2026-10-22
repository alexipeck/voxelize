# Voxelize live scene: specification and plan

**Revision 2026-10-22 (j).** Replaces the 2026-10-20 "High-Throughput Live Voxel Scene Implementation Plan". The `live` branch implements §2–§7, except where §2.6 and §8 say a change is pending. §8 lists the decisions, both made and open, and §9 is the work that follows them.

Decided so far:

- Voxel size is a runtime parameter. The vehicle is about 55–60 cm wide and keeps clear of obstacles, so 0.2–0.25 m is the likely operating range; 0.1 m is probably finer than needed.
- Endpoint quantization and path caching are dropped.
- Every ray counts. Each voxel stores hit and miss votes (`VoteModel`), and the hit share works as a probability.
- The planner's blocked rule is configurable (`BlockedRule`). The intended operating mode is the safest one: any hit means blocked.
- Results are deterministic and, by default, independent of ray order.
- Publish once per completed sensor pass.
- Track changes only to trigger a replan when the planned path is affected.

**How to read it.** MUST means required and covered by a test. SHOULD is the default; deviate only with a measurement and say so in the change. Each rule appears once. Numbers marked *host* come from an Apple M4 and say nothing about the board.

## 1. Context and cost model

- `voxelaz` keeps its offline LAS/LAZ API and `voxelize` CLI. The live scene is `voxelaz::live` and does not use the LAS types.
- **Input:** points already registered in one world frame (after LIO, deskew and extrinsics). Each point comes with the sensor origin that observed it, or with one origin per scan. The library does no registration and takes no timestamps.
- **Consumer:** a path planner (A* or similar) on another thread, reading voxel states around the vehicle. It does not track which chunks changed. It needs to replan when an update affects the path it is flying (§6).
- **Board:** Radxa ROCK 5T, RK3588 (4× Cortex-A76 up to 2.4 GHz, 4× Cortex-A55). The board is shared with LIO, flight control and the planner.
- **Sensor rate:** 2.3 M returns/s, i.e. about 230k returns per scan at 10 Hz.
- **Sensor pattern:** roughly a hemisphere per pass, one pass every 100 ms; other sensors complete a pass faster. The synthetic sensor defaults to a ±22.5° band; `bench_scan --elevation-min/--elevation-max` sets its field of view.

**Cost model.** A ray crosses about as many voxels as the L1 distance from source to hit, measured in voxels; that is about 1.4× the Euclidean range divided by the voxel size. One writer on one M4 core does 70–89 M cell updates/s (*host*) at any voxel size, so the ray rate is roughly that divided by cells per ray.

Synthetic warehouse, 40 m range, two u32 votes per voxel, *host*:

| Voxel | Cells/ray, band ±22.5° | Cells/ray, upper hemisphere | M rays/s, band | M rays/s, hemisphere | Publish p50 per pass | MB per map copy after about 8 s |
|---|---|---|---|---|---|---|
| 0.1 m | 178 | 150 | 0.41 | 0.49 | about 7 ms | about 140 |
| 0.2 m | 89 | 75 | 0.87 | 1.02 | 0.9 ms | about 25 |
| 0.25 m | 72 | 60 | 1.07 | 1.25 | 0.5 ms | about 14 |
| 0.4 m | 45 | 38 | 1.62 | 1.88 | 0.2 ms | about 6 |

At 0.25 m with a hemisphere sensor, one host writer is about 1.8× short of 2.3 M rays/s. A Cortex-A76 will do fewer. That leads to the following targets:

- **Point modes:** 2.3 M points/s. Met on the host with a wide margin (54 M/s max at 0.2 m; a few ms of CPU per 230k-point scan).
- **Ray mode:** a ray target MUST name a voxel size and range distribution, and SHOULD be stated in cell updates/s. At 0.2–0.25 m, 2.3 M rays/s probably needs shard writers (D4) and store-path speedups (§9 Phase B). D11 is an extra lever. The board numbers decide how many of these are needed (D8).
- **Where the time goes (*host*):**
  - About 80% of each cell update is the store and model write. Bare DDA stepping runs at about 280–310 M cells/s.
  - Publishing once per pass costs well under 1 ms at 0.2–0.25 m, and about 7 ms at 0.1 m with votes (D12).
  - Memory scales with 1/voxel³: going from 0.1 m to 0.25 m cuts it about 10× (table above).

## 2. Invariants (MUST)

1. **Ownership.** Each chunk has exactly one owning writer. Insert and publish take `&mut self`. A writer is `Send` but not `Clone`. Ingestion takes no lock, atomic or shared write per point, ray or voxel. A multi-writer design (D4) keeps this rule: one owner per chunk.
2. **Visibility.** Readers see published generations only. A generation contains every observation inserted before its `publish` call and none inserted after. Readers never run the update model.
3. **Unknown is not zero.** An unobserved voxel reads as `None`. An observed voxel stays observed whatever its value, including `0.0`.
4. **Rejection writes nothing.** Invalid coordinates, range-filtered observations and capacity rejections make no voxel updates and are counted in `BatchStats`. `max_chunks` and `max_cells_per_ray` are checked before a ray's first update.
5. **Range filter.** The filter tests the Euclidean distance from the observation's own sensor origin to the hit, with inclusive bounds, compared squared. Rejecting the hit rejects the whole ray. With a filter configured, point inserts that carry no source fail with `RangeFilterRequiresSource` and write nothing.
6. **Determinism and order.**
   - The same input and configuration MUST give a bit-identical map, on the host and on the board.
   - The default model SHOULD be commutative, so the map also does not depend on ray order. Integer vote counts (D1) are commutative.
   - Every ray counts: no mode may drop a ray's evidence. Modes may merge evidence, but only as weighted votes.
   - Order-sensitive models stay possible, for example clamped log-odds or f32 sums. For those, updates to any one cell are applied in input order. Measured: shuffling rays within each scan changes the class of 0.004% of voxels under the reference clamp at 0.1 m, but those cells swing the full range.
   - Registration timestamps are handled upstream and are not the reason for any of this.
   - *Pending:* the current default reference model is clamped log-odds. Switching the default to votes is D1.
7. **Offline compatibility.** The offline exports stay the same: `VoxelGrid`, `VoxelPoint`, `VoxelizeConfig`, `voxelize`, `CancellationToken`, `ClassificationFilter`, `VoxelizeError`, `Result`, `CHUNK_SIZE` and `BOUNDED_QUEUE_CAPACITY`. The CLI output point set also stays the same. Output point order already varies between runs (HashMap iteration), so outputs are compared as sets.
8. **Gates.** These pass: `cargo test --all-targets`, `cargo clippy --all-targets -- -D warnings`, `cargo fmt --all -- --check` and `cargo +1.95.0 check --all-targets`. They all pass at this revision; the offline lint fixes did not change behaviour.
9. **Evidence.** No throughput claim about the board without a `scripts/bench_board.sh` run on the board.

## 3. Geometry and traversal (MUST)

- **Voxel mapping.** `VoxelGeometry { origin, voxel_size }` maps a world point to `floor((world - origin) / size)` per axis, so cells are half-open. Keys are `i64`. A scaled coordinate with magnitude ≥ 2^53 is invalid and is never saturated. The chunk is `div_euclid(side)`, the local coordinate is `rem_euclid(side)`, and the local index is `x + S·y + S²·z`.
- **Ray events.** A ray is the segment from source to hit, `t ∈ [0, 1]`. Every voxel the segment crosses with positive length before the hit voxel gets one `Free`, in order. The hit voxel, `voxel(hit)`, gets exactly one `Hit`, last, and never a `Free`. A zero-length or same-voxel ray emits only the `Hit`.
- **Ties.** Tied plane crossings step together, so a voxel touched only at an edge or corner gets nothing. This is exact when the arithmetic is exact (dyadic voxel sizes, voxel-centre endpoints) and MUST be tested there. At other sizes, float rounding decides near-ties, and tests MUST NOT depend on which way they go.
- **Termination.** Traversal never steps past `t = 1`, is bounded by the L1 voxel distance plus a small slack, and allocates nothing per ray.
- **Model context.** The update model receives `RayCellContext { key, role, ray_length_m: Option<f64>, t_enter, t_exit }`. `ray_length_m` is computed once per ray, and only when `needs_metric_length()` returns true.

## 4. Storage and publication (current design)

- **Writer.** The writer holds a `Vec` of chunks, a `HashMap<ChunkKey, slot>` index and a last-chunk cache. Chunks are 16³ by default; 8³ and 32³ are available through the `SIDE` const generic. A chunk starts sparse (sorted `(local, cell)` pairs) and becomes dense (a cell array plus an observed bitset) at `promotion_threshold`, default 256. The threshold makes no measurable difference for rays.
- **Dirty set.** A writer-local `HashSet<ChunkKey>` of chunks changed since the last publish.
- **`publish()`.** Copies each dirty chunk into an immutable `Arc<PublishedChunk>` outside any lock. It then takes the index write lock once to install them, bump the generation and append to the journal. Replaced chunks are dropped outside the lock. A publish with nothing dirty leaves the generation unchanged. Dirty state is cleared only after the commit.
- **Index.** `RwLock<HashMap<ChunkKey, Arc<PublishedChunk>>>`. A snapshot copies the `Arc` handles it needs under one read lock and carries one generation. It never changes afterwards.
- **Known coupling.** A reader holds the read lock for as long as it takes to collect its chunk handles. `snapshot_all` over 200k chunks takes 7.3 ms. Two readers looping on it pushed one `publish` to 3.2 ms (median 7 µs). `publish` runs on the writer thread, so this stalls ingestion. A region snapshot around the vehicle keeps the stall small. D5 decides whether to remove the coupling.
- **Copies (double buffering at chunk level).** Two copies are resident:
  - the writer's working map, which only the ingest thread touches;
  - the published map, made of immutable chunks that readers share through `Arc`.

  `publish()` copies only the chunks that changed since the last publish and swaps them into the published index. A reader holding an older snapshot keeps that snapshot's chunks alive until it drops them, so a third, partial copy exists only for chunks that changed while a slow reader held them. Readers never wait for the writer, and the writer never waits for a reader to finish.

  This differs from classic double buffering, which swaps two whole maps and then has to bring the stale one up to date. That costs either a full copy or ingesting every pass twice, and a "lock step" swap would make the writer wait for readers. The one place the writer can still wait is the brief index write lock in `publish()`, if a reader is collecting chunk handles at that moment (the §4 coupling below). D5 (c) removes even that.
- **Memory.** Working plus published is about twice the map payload, plus the transient third copy above. At 0.25 m that is about 14 MB per copy for the warehouse window in §1. There is no eviction yet.

## 5. Public API (as implemented)

```rust
// Geometry
VoxelGeometry::new(origin: [f64; 3], voxel_size: f64) -> Result<VoxelGeometry, LiveError>;
geometry.voxel_key(world: [f64; 3]) -> Result<VoxelKey, LiveError>;
geometry.voxel_center(key: VoxelKey) -> [f64; 3];
VoxelBounds::try_new(min: VoxelKey, max: VoxelKey) -> Result<VoxelBounds, LiveError>; // inclusive

// Construction; SIDE is 16 here. PointSceneWriter::<Cell, SIDE>::open and
// RaySceneWriter::<M, SIDE>::open take 8, 16 or 32.
open_presence_scene(geometry, &PointSceneConfig) -> Result<(PointSceneWriter<Occupied>, SceneReader<Occupied>), LiveError>;
open_count_scene(geometry, &PointSceneConfig) -> Result<(PointSceneWriter<u16>, SceneReader<u16>), LiveError>;
open_ray_scene(geometry, &RaySceneConfig, model: M) -> Result<(RaySceneWriter<M>, SceneReader<M::Cell>), LiveError>;

impl<Cell: PointCell, const SIDE: usize> PointSceneWriter<Cell, SIDE> {
    fn insert_point(&mut self, point: [f64; 3]) -> Result<InsertStats, LiveError>;
    fn insert_points(&mut self, points: &[[f64; 3]]) -> Result<BatchStats, LiveError>;
    fn insert_point_from_source(&mut self, source: [f64; 3], point: [f64; 3]) -> Result<InsertStats, LiveError>;
    fn insert_points_from_source(&mut self, source: [f64; 3], points: &[[f64; 3]]) -> Result<BatchStats, LiveError>;
    fn insert_point_observations(&mut self, observations: &[PointObservation]) -> Result<BatchStats, LiveError>;
    fn publish(&mut self) -> Result<PublishStats, LiveError>;
    fn reader(&self) -> SceneReader<Cell>;
    fn memory_stats(&self) -> SceneMemoryStats;
}

impl<M: RayUpdateModel, const SIDE: usize> RaySceneWriter<M, SIDE> {
    fn insert_ray(&mut self, ray: RayObservation) -> Result<InsertStats, LiveError>;
    fn insert_rays(&mut self, rays: &[RayObservation]) -> Result<BatchStats, LiveError>;
    fn insert_rays_from_source(&mut self, source: [f64; 3], hits: &[[f64; 3]]) -> Result<BatchStats, LiveError>;
    fn publish(&mut self) -> Result<PublishStats, LiveError>;
    fn model_mut(&mut self) -> &mut M;
    fn reader(&self) -> SceneReader<M::Cell>;
    fn memory_stats(&self) -> SceneMemoryStats;
}

pub trait RayUpdateModel: Send + 'static {
    type Cell: Copy + Default + Send + Sync + 'static;
    fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext);
    fn needs_metric_length(&self) -> bool { false }
    fn initial_cell(&self) -> Self::Cell { Self::Cell::default() } // value an unknown cell starts from
}

impl<Cell: Copy> SceneReader<Cell> { // Clone + Send + Sync
    fn generation(&self) -> u64;
    fn publication_age(&self) -> Option<Duration>;
    fn get_voxel_key(&self, key: VoxelKey) -> Option<Cell>;                  // read lock per call
    fn get_voxel_world(&self, world: [f64; 3]) -> Result<Option<Cell>, LiveError>;
    fn snapshot_region(&self, bounds: VoxelBounds) -> Result<PublishedSnapshot<Cell>, LiveError>;
    fn snapshot_chunks(&self, keys: &[ChunkKey]) -> PublishedSnapshot<Cell>;
    fn snapshot_all(&self) -> PublishedSnapshot<Cell>;                       // O(chunks) under the lock
    fn changes_since(&self, generation: u64) -> Result<Changes, ChangesError>;                        // D6
    fn snapshot_changes_since(&self, generation: u64) -> Result<PublishedSnapshot<Cell>, ChangesError>; // D6
}
// PublishedSnapshot<Cell>: generation(), age(), get(VoxelKey) -> Option<Cell>, iter(), chunks()

// Votes (src/live/votes.rs)
pub struct Votes { pub hits: u32, pub misses: u32 } // 8 bytes, in 1/256 vote (Votes::UNIT); from_votes(), hit_votes(), miss_votes(), total(), hit_share()
pub enum VoteDecay { None /* default */, Exponential { keep: f64 }, CapHalving { cap_votes: u32 } }
VoteModel::new(decay: VoteDecay) -> Result<VoteModel, LiveError>; // VoteModel::default() is no decay
BlockedRule::new(min_hits: f64, min_hit_share: f64) -> Result<BlockedRule, LiveError>; // min_hits >= 0 votes, share in [0, 1]
BlockedRule::any_hit() -> BlockedRule;
rule.is_blocked(votes: Votes) -> bool; // applied by readers, never by the writer; integer shift, multiply, compare
```

**Errors.** Problems with individual observations are counted in `BatchStats`; they are not errors. `Err` covers only:

- the whole call rejected, with nothing written (`RangeFilterRequiresSource`);
- construction errors;
- an internal invariant broken mid-ray (`TraversalFailed`, `PartialUpdate`). The ray may be partly applied, and the stats so far are attached.

## 6. Integration

One bounded queue feeds one writer thread. The planner plans on a region snapshot, then revalidates its current path whenever a new generation is published:

```rust
let (mut writer, reader) = open_ray_scene(geometry, &RaySceneConfig::default(), model)?;
let planner = reader.clone();
std::thread::spawn(move || {
    let (mut path, mut seen) = (None, 0);
    loop {
        // Today the planner polls; D6 proposes reader.wait_for_generation(seen, timeout).
        let view = planner.snapshot_region(bounds_around(vehicle_pose(), 20.0)).unwrap(); // one short read lock, one generation
        if view.generation() != seen {
            seen = view.generation();
            let clear = path.as_ref().is_some_and(|p| corridor(p).all(|key| !blocked(view.get(key))));
            if !clear {
                path = plan_path(&view); // view.get(key) takes no lock
            }
        }
        wait_for_next_planning_cycle();
    }
});
for batch in incoming {
    writer.insert_rays(&batch)?;
    if publish_due() {
        writer.publish()?;
    }
}
```

**Replan trigger.** Replan when a voxel in the planned path's clearance corridor has become blocked (or unknown, if the planner treats unknown as blocked) under the planner's own rule. "A chunk changed" is the wrong trigger: free votes update the chunks along the path every scan, so it would fire on almost every publish.

The check costs one snapshot lookup per corridor voxel. A 50 m path at 0.25 m is 200 voxels, or about 3k with a 0.5 m clearance radius. At 0.1 m it is 500 voxels, or about 40k. At the measured 50–65 M lookups/s (*host*), that is under 1 ms per publish. The library only has to say when a new generation exists (D6).

## 7. Measurement

- **Harnesses.**
  - `bench_scan`: synthetic LiDAR scans; quote this one.
  - `bench_live`: repeated micro cases, for regressions only.
  - `ray_modes_study` and `vote_order_study`: compare ray semantics cell by cell, for §8.
  - `scene_render`: renders the synthetic warehouse and the vote map into `docs/images/test-scene/`. The map is drawn as blocked voxels (`--blocked-share`, `--min-hits`), as hit share (a probability colour scale) in 3D and in slices, at `--voxel` (default 0.2 m). The warehouse is analytic geometry; the simulated sensor is ray-cast against it to produce the test point clouds.
  - `decay_study`: how fast each decay mode clears a removed obstacle, and what it keeps (D7).
  - `paired_study`: how fast coarse free evidence clears the fine hit map after an obstacle leaves (D14).
  - `export_passes`: writes the synthetic data as one PLY point cloud per 100 ms pass (hit, sensor origin at fire time, time in pass), plus a `passes.csv` index.
  - On the board, `scripts/bench_board.sh` runs the `bench_scan` matrix pinned to the A76 cores. It records the board, kernel, compiler, governors, clocks and temperatures, plus a soak run, and compares a generic build with a `-C target-cpu=cortex-a76` build.
- **What a performance claim includes:** commit, board, build flags, pinning, voxel size, range, mean and p95 cells per ray, chunk side and model.
- **Build flags matter:** the library's generic code is compiled in the calling crate. Fat LTO with one codegen unit made the same ray workload about 15% faster on the host (207–211 ms instead of 243–245 ms per pass at 0.2 m). Applications SHOULD build release with `lto = "fat"` and `codegen-units = 1`, and benchmarks SHOULD report both builds. `bench_board.sh` builds both.
- **Headline figures:**
  - paced at 2.3 M/s: writer busy (one-core equivalents) and late scans;
  - max rate: input/s and cell updates/s;
  - always: publish p99, RSS, and reader lookups/s.
- Host results are in `docs/live-benchmarks.md`. No board results yet.

## 8. Decisions

| ID | Decision | Status |
|---|---|---|
| D1 | Update rule | Decided: votes (`VoteModel`), with a configurable `BlockedRule`; operating mode is any hit. Implemented |
| D2 | Scan cell dedup | Withdrawn: it drops votes, and every ray must count |
| D3 | Endpoint quantization and path caching | Dropped |
| D4 | Shard writers | Wanted; design questions below. The board script measures 1 A55 core and 2 or 4 parallel scenes on the A76 cores |
| D5 | Reader structure for the planner | Keep the current one until the board shows the stall matters, or D12 needs (d) |
| D6 | Publish per pass, replan trigger | Decided: publish once per sensor pass. Recommended: a "new generation" wait, replacing the chunk journal |
| D7 | Recency and decay | Implemented: `VoteDecay` None (default), Exponential, CapHalving, applied at publish to updated voxels. Parameters to be chosen from real flights |
| D8 | Budgets | Voxel size is runtime (0.2–0.25 m likely). CPU, ray rate and range need board numbers |
| D9 | Free-space rays without a return | Deferred |
| D10 | Memory growth | Not a constraint for the planned 50 × 50 m flights (the LiDAR sees about 130 × 130 m); eviction deferred |
| D11 | Free-space carving range | New opt-in lever: about 1.6× at 10 m in the warehouse |
| D12 | What a per-pass publish copies | Each pass dirties the whole view. Under 1 ms per pass at 0.2–0.25 m; it matters at 0.1 m or with fast sensors |
| D13 | GPU ingestion (OpenCL, Mali-G610, unified memory) | Parked: the CPU handles DDA well. Kept in reserve if compute becomes a constraint |
| D14 | Paired maps: coarse vote rays plus a half-size hit-only map | Prototyped (`PairedSceneWriter`): one generation, 8-bit sub-voxel masks, clearing from coarse free evidence. About 4% more CPU than the coarse map alone |

**Votes and order (`vote_order_study`,** 10 scans, about 2.2 M exact rays, 40 m range). "Surface cells left free" is the share of voxels that hold a return but end up free, which is false free space for a planner.

| Voxel | Clamped log-odds: classes changed by shuffling rays | Votes: map after shuffling | Free surface, log-odds | Free surface, votes blocked at hit share ≥ 50 / 20 / 10 / 5% | Pair-hint: fewer traversals / writes; classes differing from votes |
|---|---|---|---|---|---|
| 0.1 m | 0.004% (some by the full ±4 range) | identical | 39.3% | 46.3 / 27.7 / 13.6 / 3.5% | 1.81× / 1.33×; 0.06–0.09% |
| 0.2 m | 0.056% | identical | 44.7% | 48.6 / 36.6 / 22.9 / 9.0% | 3.40× / 2.18×; 0.26–0.32% |

Rerun with `cargo run --release --example vote_order_study -- <voxel_m> <scans>`.

**Earlier variants (`ray_modes_study`,** same scans, reference clamp; kept for D2 and D3):

| Voxel | Exact cells/ray | Pairs: fewer traversals / fewer cell events | Cell dedup: fewer model updates | Free surface: exact | pairs, centre | pairs, centroid | cell dedup |
|---|---|---|---|---|---|---|---|
| 0.1 m | 181 | 1.8× / 1.3× | 5.5× | 39.3% | 45.9% | 39.0% | 25.7% |
| 0.2 m | 91 | 3.4× / 2.2× | 18.2× | 44.7% | 60.9% | 44.1% | 20.0% |
| 0.2 m, floor mid-voxel | 91 | 4.2× / 2.6× | 18.1× | 51.6% | 48.5% | 49.9% | 21.1% |
| 0.4 m | 46 | 9.4× / 5.7× | 64.4× | 40.9% | 65.9% | 40.5% | 12.1% |

The default grid puts the floor on a voxel boundary; the mid-voxel row shifts the grid by 0.1 m. The scene is synthetic: flat floor and walls, boxes, and uniform ±2 cm range noise, with no spurious returns or moving objects.

**D1 — Update rule: votes (decided).** Store two integers per voxel, hits and misses (`u32` each, 8 bytes per voxel). Each ray adds one miss to every voxel it crosses and one hit to its endpoint voxel. The hit share, hits ÷ (hits + misses), works as the voxel's probability of being blocked.

- **Why:** addition commutes, so the map is bit-identical whatever the ray order (measured above). It is deterministic, and every ray counts.
- **Read-time rule:** the planner decides "blocked" from the two counts, for example "hits are at least 10% of the votes". It can change that rule without rebuilding the map.
- **Erosion:** grazing rays cross voxels that other rays hit, so surfaces erode under any per-ray rule. With votes, the threshold controls how much. At 0.1 m, the surface left free falls from 46% (blocked at a 50% hit share) to 3.5% (at 5%). The clamped log-odds reference sits at 39%.
- **What those figures overstate:** the metric counts every voxel that holds a return. On a flat surface, many of those voxels are mostly free space. Either the surface lies on a voxel boundary and range noise puts a return just above it, or the voxel straddles the surface. Calling such a voxel free is often right (see `docs/images/test-scene/map-slice-floor.png`). For thin obstacles (a pole, cable or branch thinner than a voxel), any return puts a hit in that voxel, so an "any hit" or "at least N hits" rule keeps it blocked. A pure hit-share rule can lose it: rays passing beside the object inside the same voxel add misses, and larger voxels make that more likely. So the planner's rule SHOULD include a hit-count floor, for example blocked if hits ≥ N or the hit share is at least x, with D7 decay so that transient returns fade.
- **Choosing the threshold:** the synthetic scene has no dust, rain or moving objects, so a low threshold looks better here than it will in the field. Choose it on real data, possibly with a minimum hit count.
- **Costs:**
  - 8-byte cells double the cell payload compared with `f32`.
  - `u16` counters would overflow: the busiest voxel took about 14.5k misses per scan.
  - Votes never forget. A moved obstacle keeps its hits until misses outweigh them by the threshold ratio, which is slow. That is D7.
- **Implementation (done):** `VoteModel`, `Votes { hits, misses }`, `Votes::hit_share()` and `BlockedRule::new(min_hits, min_hit_share)` / `BlockedRule::any_hit()` in `src/live/votes.rs`. A test checks that the map is bit-identical when the rays arrive forwards, reversed or interleaved. `open_ray_scene` always takes an explicit model, so there is no default to change.
- **Operating mode:** any hit means blocked, the safest choice. Thin obstacles then stay blocked once anything hits them. The cost is that every stray return also stays blocked forever, however many later rays pass through the voxel: dust, rain, a person or door that has since moved, or a wall smeared by pose drift. Whether that matters depends on the environment and flight length (D7).
- **Arithmetic, no division anywhere:**
  - **Update:** one integer add per voxel, which is cheaper than a multiply and commutes.
  - **Blocked test:**
    - Any hit is `hits > 0`.
    - A hit-share threshold s = N/D is the cross-multiplied `hits·D ≥ N·(hits + misses)`. `BlockedRule` does this with s in 1/65536 steps, rounded down so it can only block more.
  - **Same thing as log-odds:** that share test equals the log-odds-style score `(D − N)·hits − N·misses ≥ 0`. A Bayesian occupancy map with independent beams has log-odds `L = l_hit·hits + l_miss·misses` (plus a prior), so any threshold on its probability is a threshold on L. That is two multiplies and an add. The counts therefore already hold the log-domain form, where multiplying probabilities becomes adding.
  - **Cost on the host:** under 1 ns per voxel for every form of the test, including f64 division (0.66 ns). Finding the voxel costs 15–20 ns. The integer `BlockedRule` takes 0.43 ns, the same as loading the cell.
  - **Graded path costs:** if A* needs a cost rather than blocked/free, an additive risk cost `−log(1 − p)` adds up along a path to `−log P(path clear)`, so A* minimises collision risk with additions. Computing p and its log for each voxel is done either lazily for the voxels A* touches (a small lookup table), or once per changed voxel at publish into a u8 cost grid (the D12 (c) class grid with 256 levels instead of 3).

**D2 — Scan cell dedup: withdrawn.** Hit-wins dedup gives each cell at most one vote per scan, which breaks "every ray counts". With votes it would also save nothing: summing votes per scan before writing costs a write per cell event, the same as writing to the map.

**D3 — Endpoint quantization and path caching: dropped.**

- **Centre-to-centre quantization** adds false free surface when a surface sits near a voxel boundary (table above).
- **Pair-hint** saves little. It traces each unique voxel pair once and weights the path by its ray count. At 0.1 m that gives 1.81× fewer traversals and 1.33× fewer writes, with voxel classes differing from exact votes in 0.06–0.09% of voxels. A hemisphere sensor sweeps the whole view each pass, so few rays share both voxels, and the per-ray pair lookup adds branching that would eat into the gain.
- **Not affected:** the writer's last-chunk cache. It is a different thing and stays, because consecutive cells of a ray usually fall in the same chunk.

**D4 — Shard writers.** Several writers, each the sole owner of a disjoint set of chunks, keep §2.1 without locks. With votes, the order in which a shard applies its updates does not change the result, which frees the routing design. Questions to settle:

1. **Partition.** Interleaved chunk ownership (for example by a hash of the chunk key) balances load. Contiguous regions do not, because rays concentrate near the sensor.
2. **Routing.** Each shard can run a coarse chunk-level traversal of the whole batch (about 16× fewer steps at side 16), then the fine DDA only inside its own chunks. Each crossing is computed from the ray origin rather than accumulated, so restarting mid-ray gives the same cells. A test MUST show that the shards together produce the same map as one writer.
3. **Publication.** Either one generation across all shards (a barrier at publish), or per-shard generations, where a planner snapshot could mix scans.
4. **Cores.** The RK3588 has 4 A76 and 4 A55 cores, shared with LIO and flight control (D8). `bench_board.sh` runs one A55 case and 2- and 4-scene parallel cases on the A76 cores; their combined rate is an upper bound for shards, including shared memory bandwidth.

**D5 — Reader structure for the planner.** Options:

- (a) Keep the RwLock index with region snapshots. That is the current design: 50–65 M lookups/s on one host core, with the §4 coupling.
- (b) Back region snapshots with a dense chunk table, for O(1) lookups.
- (c) Swap in a persistent (HAMT) published map. That gives O(1) snapshots and no reader-held lock, but costs a new dependency, and publish becomes O(dirty · log n).
- (d) Publish a derived planner grid, for example 2 bits per voxel for unknown/free/blocked, instead of raw cells. That needs the read-time rule from D1 inside the writer.

Recommendation: (a) until board numbers show the stall matters. Choose (d) if D12 needs it.

**D6 — Publish once per pass; replan trigger.**

- **Decided:** publish once per completed sensor pass. The library already works this way: the writer publishes only when the application calls `publish()`. Insert a pass's rays, in one batch or several, then call `publish()` once. A faster sensor publishes at its faster pass rate.
- **Planner:** waits for the new generation and revalidates its path once per pass (§6). The planner needs to know when to revalidate, not which chunks changed.
- **Remove:** the chunk journal: `changes_since`, `snapshot_changes_since`, `ChangesError`, `JournalConfig` and the trimming.
- **Add:** `SceneReader::wait_for_generation(after: u64, timeout: Duration) -> u64`, a condition variable signalled by `publish`. It costs nothing per insert.
- **Not recommended:** a watch list inside the writer (check registered voxels at publish). It would need the planner's blocked rule inside the writer, and revalidation already costs under 1 ms. D12 (c) gets the same effect more cheaply if it is adopted.
- **Cost:** D12.

**D7 — Recency and decay (implemented).** Votes keep all history, which has two consequences:

- **Under any hit,** a voxel that is hit once stays blocked forever, however many later rays pass straight through it.
- **Under a share threshold,** the map gets slower to change as counts grow.

`VoteModel::new(VoteDecay)` makes decay a configuration:

- **`None`** is the default.
- **`Exponential { keep }`** multiplies both counts by `keep`.
- **`CapHalving { cap_votes }`** halves both counts once their total passes the cap.

How it works:

- **The pass boundary is `publish()`.** At each publish, before the changed chunks are copied, every voxel updated since the previous publish is decayed once. Publish once per sensor pass and decay is per observed pass.
- **No ray-order dependence.** Decay runs after a pass's votes are summed, so the result does not depend on ray order within the pass. A test checks this for all three modes and three batchings.
- **Unseen obstacles are kept.** A voxel that is not observed is never decayed.
- **Integer arithmetic throughout:** counts are in 1/256 of a vote and `keep` is a 16-bit fixed-point factor.
- **Cells stay 8 bytes.** While decay is on, the model marks an updated cell with the top bit of `misses`, which is free because counts halve at 2^30 units. `publish()` visits the observed cells of changed chunks, decays the marked ones and clears the bit before copying, so readers never see it. `RayUpdateModel` gained `needs_pass_end()` and `pass_end(&mut cell)` for this; both default to doing nothing.
- **An earlier version, measured and replaced:** a separate one-bit-per-voxel "touched" array cost about 7.5% of ingest speed, because every update wrote a second cache line.
- **Power laws are not included** (1/age², 1/age³). No fixed-size cell can keep them exactly; two or three exponential accumulators per voxel would approximate one.

**Cost** (*host*, 0.25 m, upper hemisphere, 8 s, two runs):

| `VoteDecay` | M cell updates/s | Publish p50 |
|---|---|---|
| None | 74.4–75.3 | 0.57–0.58 ms |
| Exponential { keep: 0.9 } | 73.9–74.3 | 0.54 ms |
| CapHalving { cap_votes: 1000 } | 73.3–74.9 | 0.52–0.54 ms |

Decay costs about 1–2% of ingest speed, within run-to-run noise, and nothing at publish.

**Behaviour** (`examples/decay_study.rs`, 0.25 m, hovering sensor). A 1 × 1 × 1.5 m box is watched for 20 passes, then removed and watched for 60 more. The box column counts only voxels that never get a hit in a run without the box. Rules are `BlockedRule::any_hit()`, `new(0.5, 0.0)` and `new(0.0, 0.10)`.

| Decay | Rule | Passes until half the box is clear | Passes until all of it is clear | Other blocked voxels kept |
|---|---|---|---|---|
| None | any hit | > 60 | > 60 | 100% |
| None | share ≥ 10% | 38 | > 60 | 99.1% |
| keep 0.9 | any hit | > 60 | > 60 | 99.9% |
| keep 0.9 | ≥ 0.5 vote | 52 | > 60 | 99.4% |
| keep 0.9 | share ≥ 10% | 10 | 16 | 98.1% |
| keep 0.7 | any hit | 24 | 28 | 99.3% |
| keep 0.7 | ≥ 0.5 vote | 12 | 17 | 98.6% |
| keep 0.7 | share ≥ 10% | 3 | 5 | 96.2% |
| cap 1000 | any hit | > 60 | > 60 | 100% |
| cap 200 | any hit | 14 | 40 | 99.8% |
| cap 200 | ≥ 0.5 vote | 7 | 19 | 99.7% |
| cap 200 | share ≥ 10% | 4 | 5 | 98.1% |

What the table shows:

- **Under any hit,** the last fraction of a vote holds a voxel, so clearing takes about ln(256 × hits) ÷ ln(1/keep) observed passes. A small hit floor such as 0.5 vote roughly halves that.
- **Surfaces that are still there** stay 96–100% blocked. The ones that flip are voxels a surface grazes or only partly fills.
- **Still to choose from real flights:** `keep` or the cap, and the planner's hit floor.

**D8 — Budgets.**

- **Voxel size:** a runtime parameter (`VoxelGeometry::new`). 0.2–0.25 m is likely, for a 55–60 cm vehicle that keeps its distance. The chunk side is compile-time (8, 16 or 32 voxels), so a chunk spans 3.2 m at 0.2 m and 4 m at 0.25 m with side 16.
- Still open: writer CPU allowance, acceptable staleness (the publish rate), target ray rate and maximum range.

**D9 — No-return beams.** Clearing free space along a beam with no return is deferred.

**D10 — Memory growth.** There is no eviction, and the maintainer says memory is not the main constraint.

- **Planned missions:** about 50 × 50 m flight areas. With a 40 m sensor the map covers about 130 × 130 m.
- **Rough estimate:** at 0.25 m with 12-byte decaying cells, assume about 20 m of height and about 30% of that volume observed. That is about 6.5 M voxels, about 80 MB per copy before chunk overhead, and two copies. Measure on real flights.
- **Bounding it:** the range filter's maximum distance already limits how far the map extends.

The figures below are for 0.1 m; at 0.25 m divide by about 10.

- **Measured:** at 0.1 m with 4-byte cells, 15.7 M observed voxels take about 87 MB per copy, and there are two copies (working and published).
- **Estimate for the warehouse:** the 30 m × 8 m corridor cross-section is about 240k voxels per metre flown. That is roughly 2.6 MB per metre with 4-byte cells and about twice that with votes. Ten minutes at 4 m/s is about 2.4 km, so several GB.
- **Options:** drop chunks farther than R from the vehicle (a scrolling window), store all-free chunks compactly, or a hard `max_chunks` with an eviction policy.
- Outdoors, sky returns nothing and carves less free space. Measure on real data before choosing.

**D11 — Free-space carving range.** Apply the hit at the true endpoint, but add free votes only along the first R metres of each ray.

- In the warehouse (median range 11.5 m) at 0.1 m: 10 m gives 1.6× fewer cell updates, and 20 m gives 1.09×.
- Effect: space beyond R stays unknown until the vehicle comes closer.
- The gain depends on the scene; it is larger outdoors at longer ranges.
- Opt-in.

**D12 — What a per-pass publish copies.** `publish()` copies every dirty chunk. With votes, each pass changes the counts of nearly every voxel in view. So a pass dirties nearly every chunk in view, however few points it carries.

Measured (*host*, upper hemisphere, 0.1 m, 2.3 M returns/s):

| Pass rate | Dirty chunks per publish | Publish p50, votes (8 B) | Publish p50, log-odds (4 B) | Publish CPU per second, votes |
|---|---|---|---|---|
| 10 Hz | 4,106 | 7.5 ms | 4.1 ms | about 7.5% of a core |
| 20 Hz | 4,059 | 6.3 ms | 4.0 ms | about 12.7% |
| 40 Hz | 3,959 | 6.1 ms | 3.8 ms | about 24% |

With votes, that is about 135 MB copied per publish, or 1.35 GB/s at 10 Hz. The board shares its memory bandwidth with LIO, so expect worse there. Options:

- (a) Publish the raw cells every pass, as today. This is fine at 10 Hz on the host; check it on the board.
- (b) Publish every pass, but no more often than a minimum interval.
- (c) Publish a derived class grid every pass (D5 option d): 2 bits per voxel (unknown, free, blocked) instead of 64.
  - That is about 32× less to copy, and only chunks where a class actually flipped need copying.
  - The writer would update a voxel's class as its votes change (one comparison per update). It could also list the flipped voxels, which is exactly what the replan trigger needs.
  - The cost: the blocked rule lives in the writer's configuration. It could still be changed at runtime, at the price of one full reclassification. Raw votes could still be published less often, for diagnostics.

Recommendation: start with (a), measure on the board at the real pass rate, and move to (c) if publishing exceeds the budget or for faster sensors.

**D13 — GPU ingestion.** The RK3588's Mali-G610 runs OpenCL and shares memory with the CPU.

- **Why votes make this viable:** a vote is an integer add. GPU atomics in any order therefore give a map bit-identical to the CPU's. Clamped log-odds would differ from run to run.
- **Storage:** a GPU kernel needs flat, addressable storage, not the CPU's chunk hash map. A dense rolling window around the vehicle would serve: for example 64 × 64 × 16 m at 0.25 m is about 4.2 M voxels, or 34 MB of votes. It sits in unified memory, so the CPU reads it without a transfer, and scrolling it with the vehicle also bounds memory (D10).
- **Risks:**
  - Every ray starts at the sensor, so the voxels around it take an atomic add from almost every ray. That is a contention hotspot; accumulating locally per work-group, or skipping the sensor's blind zone, would mitigate it.
  - Rays differ in length (p95 is about 2.2× the mean), so lanes in a SIMD group wait for the longest ray.
  - Publishing for the planner is still a copy, or a ping-pong between two window buffers.
- **Practicalities:**
  - Check whether OpenCL works on the board's OS image (vendor `libmali`, or Mesa Rusticl with Panfrost/Panthor). `bench_board.sh` records `clinfo` and the GPU driver in `system.txt`.
  - It would need a new dependency (for example `opencl3`) behind a cargo feature, with the CPU path kept as the reference and a test comparing the two maps bit for bit.
- **Status: parked.** The maintainer has run this DDA on CPUs before and expects them to stay faster for it. The measurements agree that stepping is cheap (about 310 M cells/s on one host core) and the per-voxel store write dominates. The GPU stays in reserve if compute ever becomes the constraint.

**D14 — Paired maps (prototype).** The maintainer's proposal: plan on a coarse vote ray map, and keep a fine hit-only map at half the voxel size as a high-resolution no-go layer. Derive cheap per-voxel products from both.

`PairedSceneWriter` in `src/live/paired.rs`:

- **Two maps:** a coarse `RaySceneWriter<VoteModel, 16>` and a fine `PointSceneWriter<Occupied, 32>` at half the voxel size, with the same origin. Every accepted ray adds its hit to both. The range filter applies to both, and the fine map does no DDA.
- **Alignment is exact.** Halving the voxel size is exact in binary floating point, so a fine voxel's key divided by 2 (floor) is always its coarse parent's key; a test checks this for several origins and sizes. Chunk side 32 at half the size spans the same 3.2 m (at 0.2 m) as side 16 coarse, so both maps share chunk keys, and a coarse chunk's 4096 voxels hold exactly the 32768 voxels of its fine chunk.
- **One generation.** `publish()` decays the coarse votes, clears fine voxels, copies the changed chunks of both maps, and builds the 8-bit sub-voxel mask chunks for every changed fine chunk. It then installs all three layers under one write lock and one generation. `PairedReader::snapshot_region` / `snapshot_all` return a `PairedSnapshot` with all three layers at that generation.
- **Mask:** bit x + 2y + 4z for each hit fine sub-voxel of a coarse voxel. Its population count is the fill fraction, and the bits say where in the voxel the hits are.
- **Clearing (`ClearRule`):** the fine map has no misses, so it would never forget. A fine voxel is removed at publish when its coarse parent was observed in that pass and, after decay, has at most `max_hit_votes` hits and at least `min_miss_votes` misses. Clearing needs coarse decay: without it, a coarse voxel that was ever hit keeps its hits.

**Cost** (*host*, coarse 0.2 m / fine 0.1 m, upper hemisphere, 2.3 M returns per second, 8 s, two runs; ms per 100 ms pass):

| Build | Map | Ingest p50 | Publish p50 |
|---|---|---|---|
| default release | coarse votes only, keep 0.7 | 243.5–244.5 | 1.1 |
| default release | paired, no decay, no clearing | 204–205 | 4.2–4.3 |
| default release | paired, keep 0.7, clear (0.25, 1) | 204–205 | 7.2 |
| fat LTO, 1 codegen unit | coarse votes only, keep 0.7 | 207–208 | 1.1 |
| fat LTO, 1 codegen unit | paired, keep 0.7, clear (0.25, 1) | 210–211 | 7.2 |

What the table shows:

- **Compare like builds.** In the default build, the coarse map alone is slower than the paired maps. Its generic ray code is instantiated in the example crate rather than the library, and it optimises worse there (see §7).
- **With LTO, the paired maps add** about 1–2% to ingest and about 6 ms to each publish. That is roughly 4% more CPU than the coarse map alone.
- **Memory** grows by about 10% working and 15% published (fine hits plus masks).
- **Publish cost by part:** fine copies plus mask building take about 3 ms. Decay plus clearing take about 3 ms.
- **An earlier clearing loop** tried all 8 children of every free coarse voxel and took about 10 ms. The current one walks only the fine voxels that exist and checks their parents.

**Behaviour** (`examples/paired_study.rs`, coarse 0.2 m, hovering sensor). A 1 × 1 × 1.5 m box, repeated every 12 m, is watched for 20 passes, then removed and watched for 60 more. Box voxels are the fine voxels with no hits when the box is absent.

| Decay | Clear (max hit, min miss votes) | Box fine voxels at removal | Passes until half clear | Left at end | Of those, sharing a coarse voxel with a real surface | Real fine voxels present after removal (mean / min) |
|---|---|---|---|---|---|---|
| none | 0.25, 1 | 2082 | > 60 | 2082 | 93 | 100% / 100% |
| keep 0.9 | 0.25, 1 | 2082 | 53 | 880 | 93 | 99.9% / 99.9% |
| keep 0.9 | 0.5, 1 | 2082 | 47 | 436 | 93 | 99.9% / 99.8% |
| keep 0.7 | 0.25, 1 | 2081 | 13 | 406 | 93 | 99.5% / 99.4% |
| keep 0.7 | 0.5, 1 | 2079 | 11 | 405 | 93 | 99.2% / 99.1% |
| cap 200 | 0.25, 1 | 2082 | 19 | 985 | 93 | 100% / 100% |

What the table shows:

- **Clearing works,** but it is limited by the coarse voxel. A fine voxel cannot be cleared while its coarse parent still holds real surface; here that is 93 fine voxels, mostly floor next to the box.
- **The other leftovers** are in coarse voxels that rays cross too rarely to reach the miss threshold.
- **Surfaces still present flicker slightly.** A distant floor voxel that is hit only intermittently is cleared and later re-added. 99.1–99.9% of real fine voxels are present at any moment after the removal.
- **Not done in the prototype:**
  - per-sub-voxel weights that combine the mask with coarse votes (the reader can compute them from one snapshot);
  - incremental mask updates (masks are rebuilt per changed fine chunk);
  - a voxel size other than half;
  - `wait_for_generation`;
  - chunk deletion.

## 9. Work plan

**Phase A — Board baseline (now).**
- Run `scripts/bench_board.sh` on the ROCK 5T, with `--quick` first. The matrix defaults to 0.2 m and includes hemisphere cases at 0.2 and 0.25 m, plus 0.1 and 0.4 m comparisons.
- Deliver `summary.md`, `system.txt` and `thermal.csv`.
- Use them to settle D8, and to decide how much of D4, D3 and D11 is needed.

**Phase B — Speedups that change no semantics.** No decision needed. The target is the per-cell store write (about 80% of ray time). Candidates:
- walking runs of cells inside one dense chunk with direct indexing;
- chunk side 32 (+12% at 0.1 m on the host, but +55% memory);
- a faster hasher for the chunk index.

Acceptance: semantic tests unchanged, maps bit-identical, and `bench_scan` before/after on the board.

**Phase C — Votes model**, per D1 and D7.
- Done: `VoteModel`, `Votes`, `BlockedRule` and `VoteDecay`, with order-independence and decay tests.
- Remaining: choose the decay parameters and hit floor from real flights.

**Phase D — Shard writers**, per D4.
- Acceptance: the same-map test from D4, one generation per publish (if chosen), and a 1–4 shard scaling table from the board.

**Phase E — Replan trigger and reader**, per D6 and D5: `wait_for_generation`, journal removal.

**Phase F — Optional lever and memory**, per D11 (carving range) and D10 (memory bound). Publication changes per D12 if the board needs them.

**Visualiser (wanted, not scheduled).** Every traversed voxel is stored, not just voxels with hits; at 0.2 m after 10 passes that is 1.53 M observed voxels, 174k with hits. A visualiser should be able to show a heatmap over all observed voxels: hit share, vote totals, or misses, for example with transparency or slices. `scene_render` currently draws only voxels with hits in 3D, and slices of all observed voxels.

## 10. Acceptance checklist

- [ ] §2.8 gates pass.
- [ ] Offline CLI output sets are unchanged on a seeded LAS and LAZ fixture. This is a manual comparison today and is worth automating.
- [ ] DDA: fixture and randomized reference tests pass, including exact ties on dyadic grids, termination, and one `Hit`, last.
- [ ] Filtered, invalid and capacity-rejected observations write nothing and are counted.
- [ ] Unknown stays distinct from an observed zero, including across sparse-to-dense promotion.
- [ ] Readers see published state only; snapshots are immutable and carry one generation.
- [ ] Maps are deterministic. With a commutative model (votes), the map is bit-identical under ray shuffling. Order-sensitive models apply each cell's updates in input order. Every optional lever is off by default, and the map is bit-identical when it is off.
- [ ] Every board performance claim cites a `bench_board.sh` run with the fields in §7.

## 11. Repository baseline (checked 2026-10-21)

| Path | Role |
|---|---|
| `Cargo.toml` | `voxelaz` 0.2.0, edition 2024, MSRV 1.95.0; `las`, `clap`, `thiserror`, `anyhow`, `ctrlc` |
| `src/lib.rs` | Offline exports (§2.7) plus `pub mod live` |
| `src/voxel_grid.rs` | Offline grid. Defines `CHUNK_SIZE = 64` and `BOUNDED_QUEUE_CAPACITY = 64` (LAS buffer sizes, exported at the crate root). These are not live chunk sizes |
| `src/pipeline.rs`, `classification.rs`, `cancel.rs`, `error.rs`, `main.rs` | Offline pipeline and CLI |
| `src/live/` | `geometry`, `config`, `error`, `chunk`, `store`, `published`, `point`, `dda`, `ray`, `model`, `stats`, `bits` |
| `examples/` | `live_points`, `live_rays`, `live_observer`, `bench_live`, `bench_scan/`, `ray_modes_study` |
| `scripts/bench_board.sh` | Board benchmark matrix |

## 12. What the 2026-10-20 plan got wrong

So the next revision does not repeat it:

- It sized ray work at 5–40 cells per ray. Its own example geometry gives about 90.
- It spent its risk discussion on publication, which costs about 2% of writer time, and missed that a reader can stall `publish` through the index lock.
- Its integration example lost updates: `changes_since`, then `snapshot_chunks`, then setting the cursor to the snapshot's generation.
- It required change notification, but the consumer is a planner that does not need it.
- Its benchmark list was not binding. The implementation benchmarked one repeated ray and reported that rate.
- Some acceptance criteria could not be met: an agent cannot produce board numbers. Others contradicted each other: lint gates versus "do not touch offline code".
- Prose and signatures disagreed: the initial cell value, `ray_length_m: f64` versus "no sqrt", the missing "cursor ahead" error, and point APIs with one source per observation.
- It restated rules 3–6 times with soft verbs, which hid which ones were requirements.
