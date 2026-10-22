# Conversation Ledger

Date-ordered record of what was claimed, disputed, decided, built and tested.
Append-only: newest date last, and existing entries are never rewritten. A past
entry stands even when it turned out to be wrong — correct it with a new entry
that references it.

Format and conventions: the `journal` skill.

---

## 2026-10-21

### 15:14 · Critique of the live-scene implementation plan against the `live` branch
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

- **test** — Quality gates on `live` vs `master`.
  - evidence: `cargo test --all-targets` passes (40 unit + 1 integration). `cargo +1.95.0 check --all-targets` passes (MSRV holds). `cargo clippy --all-targets -- -D warnings` fails with 8 errors and `cargo fmt --check` fails, all in offline files (`lib.rs`, `pipeline.rs`, `voxel_grid.rs`, `cancel.rs`, `main.rs`). `master` fails the same way, so these failures were already there before the live work. The new `src/live/` code passes both.
- **claim** — The plan contradicts itself here. Phase 6 acceptance requires clippy and fmt to pass, but §1 says to document baseline failures and not rewrite offline code. The implementer left the failures in place and did not report them.
- **test** — Ran `bench_live` in release at HEAD on an M4. The numbers match `docs/live-benchmarks.md` within run-to-run noise, even though that doc says it was measured before 858a388. Ray throughput is about 68–83 M cell events/s.
- **test** — Wrote a synthetic scan benchmark in the scratchpad (not committed). Setup: 0.2 m voxels, uniform directions, one source per 230k-ray scan, publishing each scan.
  - evidence: with 1–30 m range, 89.4 cells/ray, 0.65 M rays/s, 57.9 M events/s. Publish averaged 1.9 ms (max 5.4 ms, about 1,439 chunks), which is 0.5% of writer time. With 1–10 m range, 31.9 cells/ray and 2.10 M rays/s.
  - evidence: the per-scan unique cell count is 7.5× (30 m) to 30× (10 m) smaller than the event count. Hits are placed throughout the volume rather than on surfaces, so treat the factor as indicative.
- **test** — Reader/writer coupling: with 200k published chunks, `snapshot_all` takes 7.3 ms end to end. While 2 readers loop on it, `publish` p50 stays at 0.007 ms but max rises to 3.2 ms. A reader can stall the writer thread through the index lock.
- **claim** — The plan's cost model is wrong for ray mode. It anchors on 5–40 cells/ray and puts most of its risk text on publication. Measured, per-cell DDA updates dominate (≥99% of writer time), and the plan rules out both levers that address that cost: per-scan dedup semantics and spatially sharded writers.

### 15:49 · User answers on the plan critique: quantization, ordering, sharding, readers
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 15:14 entry above.

- **rebuttal** — On ray cost: "I'm well aware of the costs of this." The user's earlier DDA project only needed line of sight, so it could stop early. Here every traversed voxel is needed.
- **decision** — The user wants endpoint quantization as an explicit opt-in feature: snap source and hit to voxel centres, collapse to unique voxel→voxel pairs, and run one full DDA per pair. Its precision effects must be discussed before it is built.
- **note** — The user asked whether that is what I meant by "merging duplicates". It is not. I meant collapsing cell updates within a scan (OctoMap style, exact geometry). Theirs collapses rays and trades geometric precision. The two can be combined.
- **rebuttal** — The user's reading of "do not reorder" was: "to ensure that the time we have for each point is correct so that it can be registered correctly".
- **decision** — Spatially sharded writers, each owning separate chunks: the user likes the idea. Design to be discussed before building.
- **decision** — Change notification is not needed. The reader runs voxel path planning (possibly A*) and does not track what changed.
- **note** — The user will have the ROCK 5T within a few days.
- **decision** — Authorised to fix the other problems from the critique. Anything the user discussed, or anything that changes precision, comes back for discussion first.
- **test** — Endpoint quantization vs. exact rays, measured on the synthetic warehouse scan (`examples/bench_scan/scene.rs`). Setup: 10 scans of about 221k returns, log-odds ±0.85/−0.4. A "surface cell" is any voxel that contains a return.
  - evidence: at 0.2 m, voxel→voxel pairs mean 3.4–4.2× fewer traversals but only 2.2–2.6× fewer cell events, because long rays rarely share a pair. At 0.4 m it is 9.4× fewer traversals and 5.7× fewer events. At 0.1 m, 1.8× and 1.3×.
  - evidence: surface cells left free after 10 scans:
    - exact per-ray: 39–52%
    - pairs traced centre-to-centre: up to 61–66%, worst when a surface lies on a voxel boundary
    - pairs traced to the centroid of their sources and hits: within 1 point of exact
    - per-scan cell dedup with hit-wins, exact geometry: 12–26%, with 5.5–59× fewer model updates
  - evidence: DDA stepping alone runs at about 3.4 M rays/s (about 310 M cell steps/s). The full library path runs at 58–71 M cell events/s. So about 80% of per-cell cost is the store/model update, not stepping. The promotion threshold (1/64/256) makes no difference for rays.
- **claim** — Grazing-angle surface erosion comes from the update rule, not from DDA precision. Even exact per-ray traversal frees 40–50% of surface cells. This should go to the user as a formula decision, ahead of quantization.
- **decision** — Fixed the pre-existing clippy and fmt failures in the offline code without changing behaviour. Changes: `#[allow(clippy::too_many_arguments)]` on two pipeline functions, `is_multiple_of`, `div_ceil` on a `u32`, dropped a `.clone()` on a `Copy` type, struct-literal `Point` init, plus `cargo fmt`. The user authorised fixing the non-design problems.
  - evidence: CLI outputs before and after were compared by header and sorted point set on a seeded 300k-point LAS and LAZ fixture: voxel sizes 0.25 and 0.7, and with `--counts-as-intensity --include-classification 2,3,5`. All 4 match. Byte comparison isn't possible because output point order varies from run to run (HashMap). `clippy -D warnings` and `fmt --check` are now clean.
- **decision** — Added a board benchmark: `examples/bench_scan` (synthetic warehouse scans, paced or max-rate, per-thread CPU and RSS from /proc, publish percentiles, an A*-style reader) plus `scripts/bench_board.sh` (core pinning, system and thermal capture, generic vs `cortex-a76` builds, a soak run). The plan required ROCK 5T numbers, which an agent cannot produce, so this replaces that criterion with a script the user runs.
- **test** — `bench_scan` host matrix on the M4 (8 s per case, 0.2 m, 40 m, 2.3 M returns/s sensor). Point presence 53.8 M/s. Rays 0.85 M/s at 89 cells/ray (77 M events/s). Paced at 2.3 M/s, the ray writer was late on 79 of 80 scans. At 0.5 M/s it kept up. Voxel 0.1 m: 0.42 M rays/s, publish p99 11.6 ms. Voxel 0.4 m: 1.66 M rays/s. Region-snapshot reader: 50–65 M lookups/s. The `/proc` CPU and RSS paths and the shell script are untested here (no Linux machine available).
- **decision** — Wrote `docs/live-scene-plan.md`, replacing the 2026-10-20 plan. It states each invariant once as MUST/SHOULD, gives the measured cost model, and documents the API as implemented. It corrects the integration example and adds a board measurement procedure. The user's points and anything that changes precision are listed as open decisions D1–D9 with options and recommendations, not implemented: update rule/erosion, scan cell dedup, endpoint quantization, shard writers, reader structure, dropping the journal, recency, budgets, no-return beams.
- **deviation** — Small library change: `ray.rs` lost two unreachable error branches. An internal traversal failure now always returns `Err` with stats, never `Ok` counted as invalid. The `TraversalFailed` message now says the ray may be partly applied.
- **test** — Final gates on the working tree: fmt, clippy `-D warnings`, `cargo +1.95.0 check --all-targets` and `cargo test --all-targets` (40 + 1) all pass. Nothing is committed.

### 18:57 · Follow-up: quantization dropped, 0.1 m voxels, order-independent votes, path-change trigger
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 15:49 entry above.

- **verdict → I was right** — Centre-to-centre endpoint quantization erodes surfaces. User: "I do see the issue with my endpoint quantization idea. Maybe we won't do that."
- **decision** — Voxel size is 0.1 m for now (user).
- **note** — The user asked whether the path traced for one voxel→voxel pair can be reused as a hint for another ray with the same pair.
- **decision** — Each ray must still count. The user framed hits and frees as votes: "collapsing into fewer results could be considered as votes blocked vs votes free. But we still need each to count."
- **rebuttal** — On ordering: "I don't think it should matter what order we process the points in, they should get roughly the same result regardless of the order within reason depending on what we store for each voxel... I do think it should be deterministic."
- **decision** — Change notification is not needed in general. The use case that does need it: once a path is planned, an update on that path should trigger a replan.
- **test** — Order sensitivity and vote counts (`examples/vote_order_study.rs`, 10 scans, about 2.2M exact rays).
  - evidence: clamped log-odds with rays shuffled within each scan changes the class of 0.004% of voxels at 0.1 m (0.056% at 0.2 m), but some cells swing the full range (8.0). Integer votes (hits, misses) give a bit-identical map under shuffling.
  - evidence: surface cells left free at 0.1 m: clamped log-odds 39.3%. Votes with blocked at a hit share of ≥50 / 20 / 10 / 5%: 46.3 / 27.7 / 13.6 / 3.5%. Spurious returns are not modelled, so low thresholds look better here than they will in the field.
  - evidence: the user's pair-hint with votes (each unique voxel pair traced once along its first ray, weighted by its count) gives 1.81× fewer traversals and 1.33× fewer cell writes at 0.1 m (3.4× / 2.18× at 0.2 m). Voxel classes differ from exact votes in 0.06–0.09% of voxels at 0.1 m.
  - evidence: the most-crossed voxel took 145k misses in 10 scans (about 14.5k per scan), so u16 counters would overflow within one scan. u32 lasts hours.
- **test** — `bench_scan` at 0.1 m, 20 s max rate: 178 cells/ray mean, p95 395. Chunk side 16: 79 M events/s (0.44 M rays/s), publish p99 5.7 ms, about 87 MB working + 87 MB published for 15.7 M observed voxels. Side 32: 89 M events/s (+12%) but 135 MB per copy (+55%). The two-u16 evidence cell runs at the same speed as f32.
- **test** — Capping how far each ray marks free space, at 0.1 m in the warehouse (median range 11.5 m): 10 m cap gives 1.6× fewer cells, 20 m gives 1.09×. Small lever in an enclosed scene.
- **claim** — At 0.1 m the target needs about 416 M cell events/s, 4.7–5.3× one M4 core. Expected levers: shard writers (≤ cores), store-path work, pair-hint 1.3×, side 32 1.12×. Board numbers are needed before choosing.
- **decision** — Updated `docs/live-scene-plan.md` to revision (b). Decided: 0.1 m voxels, centre quantization dropped, every ray counts, deterministic and order-independent by default. Recommended: votes as the default model (D1), and a `wait_for_generation` replan trigger in place of the chunk journal (D6). Optional: pair-hint (D3). Withdrawn: scan cell dedup (D2). New decisions: memory growth at 0.1 m (D10) and free carving range (D11). Nothing in the library changed.
- **decision** — `bench_board.sh` now defaults to 0.1 m (`--voxel` overrides), with a 0.2 m comparison case. A paced `bench_scan` run now also stops at its wall-clock length and reports the backlog. Before this, a writer that could not keep up made the case run several times longer. The host paced row was refreshed: 31 of 32 scans late at 0.2 m.
- **test** — Gates after these changes: fmt, clippy `-D warnings`, MSRV 1.95.0 check and `cargo test --all-targets` all pass. Nothing is committed.

### 19:30 · Follow-up: runtime voxel size, caching dropped, scene renders, publish once per pass
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 18:57 entry above.

- **rebuttal** — On 0.1 m: "0.1m voxels is not a hard limit, it should be parameterizable at runtime, it's just what I want to use." The plan's "decided" wording overstated it. Voxel size is already a runtime parameter (`VoxelGeometry::new`); chunk side is compile-time.
- **decision** — The user drops path caching (pair-hint): "caching in this case may be futile because the sensor is scanning approximately an entire hemisphere per pass. I know that caching causes quite a lot of branching."
- **note** — The user's sensor covers roughly a hemisphere per pass, every 100 ms. The synthetic sensor so far is a ±22.5° band.
- **decision** — The user proposes publishing once per completed pass, with the planner checking its path once per pass. A faster sensor would publish and check at its own faster pass rate.
- **note** — The user asked for screenshots of the test environment.
- **test** — Field of view at 0.1 m with u32 votes (`bench_scan --model votes`, max rate). Band ±22.5°: 178 cells/ray, 72 M events/s, publish p50 7.2 ms over about 4,600 chunks, 144 MB per copy. Upper hemisphere (0–90°): 150 cells/ray, publish p50 7.2 ms, about 4,100 chunks. Lower hemisphere (−90–0°): 58 cells/ray, 1.24 M rays/s, publish 1.3 ms.
- **test** — Publish cost per pass with an upper hemisphere at 0.1 m, at the same 2.3 M returns/s. Votes: about 4,100 / 4,060 / 3,960 dirty chunks and p50 7.5 / 6.3 / 6.1 ms per publish at 10 / 20 / 40 Hz passes. Log-odds: about 4.1 / 4.0 / 3.8 ms. A pass dirties almost everything in view however few points it has, so publish CPU per second grows linearly with pass rate (votes on the host: about 7.5% → 12.7% → 24% of a core).
- **verdict → partly wrong (mine)** — My "surface cells left free = dangerous false free space" metric overstates the risk. The scene renders show why. With the floor exactly on a voxel boundary, the layer above it is genuine free space; a return lands there only through noise. With the floor mid-voxel, the voxel is partly free space. The dangerous case is thin obstacles thinner than a voxel, and the synthetic scene has none yet. Votes with a read-time threshold still give the planner the control.
- **decision** — Added `examples/scene_render.rs` and its output in `docs/images/test-scene/`: top view, two perspective views (band and upper hemisphere) and two 0.1 m vote-map slices. `bench_scan` gained `--elevation-min/--elevation-max` and `--model votes`.
- **decision** — Plan revision (c). Voxel size is a runtime parameter with 0.1 m as the operating value. D3 (quantization and path caching) is dropped. D6 is now "publish once per sensor pass" (already how `publish()` works) plus the `wait_for_generation` recommendation. New D12 covers per-pass publish cost: (a) raw cells, (b) a minimum interval, (c) a 2-bit class grid that copies only chunks whose class flipped and lists the flipped voxels for the replan trigger; recommend (a), then (c) if the board needs it. D1 now notes the thin-obstacle caveat. Library code unchanged; gates pass; nothing committed.

### 22:21 · Follow-up: thin obstacles, larger voxels, voxel renders, per-pass test data
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 19:30 entry above.

- **rebuttal** — On thin obstacles: "if a point hits within one of those voxels, it still becomes a member of that voxel. So in theory, it's fine as long as it's something hits it." True in point mode and under an "any hit" or "at least N hits" rule. In ray mode, under a hit-share rule, rays passing beside a thin object inside the same voxel add misses that can outvote its hits.
- **decision** — The vehicle is about 55–60 cm wide and should not fly close to anything, so 0.1 m voxels may be excessive. The user is considering 0.2–0.25 m, possibly larger. Voxel size stays a runtime parameter.
- **note** — The user observed that none of the rendered images showed a voxel scene. They took the warehouse to be a generator of test point clouds. The last two images were in fact 2D slices of the voxel map.
- **decision** — Test data should be a point cloud split into 100 ms sensor passes, matching the real sensor's samples.
- **test** — Voxel size with u32 votes on the host (8 s, max rate). Band / upper hemisphere: 0.2 m 89 / 75 cells/ray, 0.87 / 1.02 M rays/s, publish p50 about 0.9 ms, about 25 MB per copy. 0.25 m 72 / 60 cells/ray, 1.07 / 1.25 M rays/s, publish p50 about 0.5 ms, about 14 MB. 0.4 m 1.62 / 1.88 M rays/s. At 0.25 m with a hemisphere sensor, one host writer is about 1.8× short of 2.3 M/s.
- **decision** — Added `examples/export_passes.rs`: one binary PLY per 100 ms pass (x y z hit, sx sy sz sensor origin at fire time, t in pass) plus `passes.csv`. Checked on 3 hemisphere passes: about 226k returns each, 6.3 MB, ranges 1.5–40 m, t 0–0.1 s. `scene_render` now also draws the vote map as 3D voxels (`map-voxels.png`, 0.2 m) and takes a voxel-size argument.
- **decision** — Plan revision (d): 0.2–0.25 m is the likely voxel size, and the cost-model table now covers 0.1, 0.2, 0.25 and 0.4 m. D1 now recommends a hit-count floor in the planner's rule so thin obstacles stay blocked. D10 and D12 are de-emphasised at 0.2–0.25 m. `bench_board.sh` defaults to 0.2 m and adds hemisphere vote cases at 0.2 and 0.25 m. Gates pass; library code unchanged; nothing committed.

### 22:40 · Follow-up: configurable threshold, hit-share images, copies, board GPU
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 22:21 entry above.

- **decision** — The 10% hit-share threshold must be configurable.
- **decision** — The user wants an image of hit share per voxel as a scalar field, "this will basically become a probability".
- **verdict → I was right** — On thin obstacles, the user agreed that rays through a voxel count as misses: "you are right that lines that go through the voxel count as a non-hit". Their operating mode is the safest one: blocked if there is any non-zero chance of a hit, i.e. any hit.
- **note** — The user asked whether "MB per copy" means keeping two or three copies of the scene, like double buffering where the latest can be read while processing continues in lock step. Requirement: nothing should truly block ingestion, but reads must be possible.
- **note** — The board has 4 performance (A76) and 4 efficiency (A55) cores, plus a GPU that runs OpenCL, with unified memory.
- **decision** — Added `VoteModel`, `Votes { hits, misses }` (`hit_share()`) and `BlockedRule::new(min_hits, min_hit_share)` / `BlockedRule::any_hit()` to the library (`src/live/votes.rs`, new `LiveError::InvalidBlockedRule`). The 10% threshold is now configurable here and in `scene_render` (`--blocked-share`, `--min-hits`). This is the first library change since the critique, made because the user settled on votes, hit share as a probability, and a configurable rule.
  - evidence: 3 new tests: rule thresholds and validation, saturation, and maps bit-identical when rays arrive forwards, reversed or interleaved. 43 unit + 1 integration tests pass; fmt, clippy `-D warnings` and MSRV 1.95.0 pass.
- **test** — Hit share at 0.2 m after 10 passes: 174k of 1.53 M observed voxels have hits; share p10 5.3%, p50 53.7%, p90 100%. Spread is wide enough for a linear colour bar. New images: `share-voxels.png`, `share-slice-1m.png`, `share-slice-floor.png`.
- **decision** — Answered the copies question in plan §4. There are two resident copies (working and published), double-buffered at chunk level by copying only changed chunks on publish. A slow reader keeps a transient third copy of only the chunks that changed while it held them. Readers never block inserts. The only remaining wait is the brief index write lock in publish (D5 (c) would remove it). Whole-map lock-step double buffering would need a full copy or double ingestion, and would make the writer wait for readers.
- **decision** — Added plan D13 (GPU ingestion via OpenCL on the Mali-G610, unified memory). Votes make GPU atomics deterministic. A rolling dense window would give flat storage. Risks: contention at the sensor and ray-length divergence. Prototype only if the CPU falls short. `bench_board.sh` now records the GPU driver and `clinfo`, runs one A55 case and 2- and 4-scene parallel cases on the A76 cores. Its helpers were tested with a fake `taskset`; the full script is still untested on Linux.

### 22:50 · Follow-up: every traversed voxel, heatmaps, division-free arithmetic
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 22:40 entry above.

- **note** — The user pointed out that the scene images only drew voxels with hits, though every traversed voxel takes part in the calculation. The map does store every traversed voxel; only the renders filtered them. At 0.2 m after 10 passes there are 1.53 M observed voxels, 174k of them with hits. A render is not needed now, but a heatmap would be wanted in a visualiser.
- **note** — The user asked whether cells can be kept so that updates use multiplication and path planning uses multiplication or other simple arithmetic, instead of the division that hits ÷ (hits + misses) implies.
- **test** — Cost of a blocked test per voxel on the M4 (16 M cells, black-boxed loads). Load only 0.43 ns. Any hit 0.66 ns. Integer cross-multiply 0.42 ns. Old f64 `BlockedRule` 0.98 ns. f64 division 0.66 ns. Integer log-odds score 0.41 ns. All variants agree on 221,719 blocked cells. Map lookup is about 15–20 ns, so division is not the bottleneck for path planning.
- **decision** — `BlockedRule::is_blocked` is now integer only: `hits << 16 >= floor(share · 65536) · total`, rounding the threshold down so it can only block more. It measures 0.43 ns.
  - evidence: new test `integer_rule_matches_exact_ratios` checks shares 1/10, 3/10, 1/4, 1/2 and 1/20 against exact rationals for every hits, misses in 0..300. Saturated counts are covered. 44 unit + 1 integration tests pass; clippy, fmt and MSRV pass.
- **claim** — No division is needed anywhere. Updates are integer adds. Any threshold on hit share equals a linear log-odds score `(D−N)·hits − N·misses ≥ 0`, so votes already hold the additive log-domain form. Graded A* costs should be additive `−log(1−p)`, precomputed per changed voxel at publish or computed lazily with a lookup table. Recorded in plan D1, with u16 counters plus halving decay in D7 and the heatmap visualiser in §9.

### 23:00 · Follow-up: why decay; GPU kept in reserve
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 22:50 entry above.

- **rebuttal** — "Why are you suggesting adding decay? As in, I presume that this is over time that that matters less."
- **verdict → both partly right** — Decay is about old evidence mattering less, but driven by new evidence rather than by the clock. Under any-hit, a voxel hit once stays blocked forever, however many later rays pass through it. Moved objects, stray returns and pose-drift smears therefore accumulate as permanent blocks. My plan wording ("a requirement") was too strong: in a static scene, with short flights and good LIO, it may not be needed. It depends on the environment.
- **decision** — The GPU stays in reserve. User: "I don't believe it'll be more efficient to use the GPU. I've done this exact DDA math on CPUs, and they are very good at it." It is there if compute ever becomes the constraint. This fits the measurements: DDA stepping alone runs at about 310 M cells/s on one host core, and the per-voxel store write is about 80% of the cost.
- **decision** — Plan revision (g). D7 now explains decay as evidence-triggered and conditional on the environment. It recommends cap-triggered halving if decay is needed, because it only forgets voxels still being observed. Time-based decay is not recommended, because it forgets obstacles out of view. D13 (GPU) is parked in reserve. No code changed.

### 23:10 · Follow-up: decay representation and modes; memory and flight area
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 23:00 entry above.

- **decision** — "maybe decay is necessary." The user asked whether a different stored value would update more efficiently with decay, allowing exponential decay "such as inverse square or inverse cubed". If cap-halving is better, they suggested configurable decay modes.
- **note** — Memory is not the main constraint. The flight area is about 50 × 50 m, but the LiDAR will see a much larger area.
- **claim** — Inverse-square and inverse-cube are power laws, not exponentials. Only exponential decay can be kept in a fixed-size cell with one multiply per step. A power law needs the vote history, or an approximation as a sum of two or three exponentials.
- **decision** — Prototyped decay in the examples, not the library: `examples/bench_scan/decay.rs`. Counts are fixed-point (1/256 vote) with a per-voxel pass stamp, and decay is applied on a voxel's first update in each pass. Modes are `none`, `exp:keep` and `cap:votes`, configurable as the user suggested. Power-law decay (1/age², 1/age³) is not offered: it cannot be kept in a fixed-size cell and would need two or three exponential accumulators.
- **test** — Ingest cost on the host at 0.25 m, upper hemisphere, two runs. `VoteModel` 74.6–74.9 M events/s; prototype `none` 70.4, `exp:0.9` 70.3–70.8, `cap:1000` 70.6–71.0. Publish p50 0.52 ms → 0.65–0.71 ms. The decay maths is free; the 12-byte cell costs about 5–6% ingest and about 30% publish.
- **test** — `examples/decay_study.rs` at 0.25 m: hovering sensor, 1 × 1 × 1.5 m box present for 20 passes then removed for 60. Box voxels are those with no hits when the box is absent.
  - evidence: without decay, any-hit never clears the box and a 10% share rule clears half in 38 passes. With `exp:0.7`, any-hit clears all in 29 passes, a 0.5-vote floor in 18, and the share rule in 5. With `exp:0.9` and any-hit, not within 60 passes. With `cap:200` and any-hit, 41 passes.
  - evidence: other blocked voxels kept at the end: 96.2–100%.
  - evidence: the first version of the study wrongly counted floor voxels next to the box; it was fixed with a no-box reference run before these results were taken.
- **decision** — Plan revision (h): D7 now carries the modes, cost and behaviour tables, and recommends library-level configurable decay defaulting to `none`. D10 is deprioritised: the user's flights are about 50 × 50 m (the LiDAR covers about 130 × 130 m), and memory is not the constraint. Gates pass; nothing committed.

### 23:33 · Decay modes moved into the library
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 23:10 entry above.

- **decision** — The user asked to "Move the decay modes into the library as a configuration on the vote model (none, exponential, cap-halving; default none), with the pass boundary at publish, and keep cells at 8 bytes if practical."
  - Library: `VoteDecay::{None (default), Exponential { keep }, CapHalving { cap_votes }}` and `VoteModel::new(decay)`. `Votes` is now in 1/256 vote (`Votes::UNIT`, `from_votes`, `hit_votes`, `miss_votes`); `BlockedRule::new(min_hits: f64 votes, share)`; new `LiveError::InvalidVoteDecay`.
  - `RayUpdateModel` gained `needs_pass_end()` and `pass_end(&mut cell)`, both defaulting to no-op. `RaySceneWriter::publish()` calls `pass_end` for the observed cells of dirty chunks before copying them. `WorkingChunk::for_each_observed_mut` supports this.
  - The examples use the library model (`bench_scan --decay`); the prototype `examples/bench_scan/decay.rs` was deleted.
- **deviation** — Semantics differ slightly from the prototype: decay is applied after the pass's votes (at publish) rather than before them. Share-based results are identical; absolute-hit rules clear about one pass sooner.
- **test** — First library design: a separate 1-bit-per-voxel touched array. Measured: none 73.2–73.9 M events/s; exp:0.9 67.6–68.3; cap:1000 67.5–68.9 (about −7.5%). Publish p50 about 0.55 ms in every mode.
- **decision** — Replaced it with a flag in the top bit of `misses`. Overflow halving moved from 2^31 to 2^30 units (about 4.2 M votes), so the bit is free. `publish` clears the flag before copying, so readers never see it. Cells stay 8 bytes, with no extra memory.
  - evidence: none 74.4–75.3 M events/s; exp:0.9 73.9–74.3; cap:1000 73.3–74.9, about 1–2% and within noise. Publish p50 0.52–0.58 ms in every mode.
- **test** — 48 unit tests pass (chunk visit across promotion; validation; 8-byte cell; overflow halving and flag clearing; decay once per publish and only for updated voxels; cap only above the cap; maps identical under three ray orders and batchings for all three modes, with the flag never published), plus the integration test. fmt, clippy `-D warnings` and MSRV 1.95.0 pass.
  - evidence: `decay_study` with the library: keep 0.7 with any hit clears the box in 28 passes, the 0.5-vote floor in 17, share 10% in 5; other surfaces kept at 96.2–100%. `scene_render` output is pixel-identical to before the unit change.
- **decision** — Plan revision (i): D7 is marked implemented, with the cost and behaviour tables; the README and API section are updated. Nothing committed.

## 2026-10-22

### 14:46 · Commit of the session's work
`session b8a64a31` · `live` @ `858a388` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 23:33 entry on 2026-10-21.

- **decision** — The user asked to "Commit the working tree changes with a sensible message." Everything from the session goes in as one commit on `live`: library votes and decay, example harnesses and studies, the board script, docs, images, legend and this ledger. Chat transcripts stay out (their folder is git-ignored).
- **test** — Before committing: fmt, clippy `-D warnings`, `cargo +1.95.0 check --all-targets`, 48 unit + 1 integration tests and `bash -n scripts/bench_board.sh` all pass. All untracked files' modification times match edits made in this session.
- **deviation** — Pushing `live` (37f832a) from this session failed for lack of credentials. Over HTTPS there is no stored GitHub login and no `gh`. Over SSH, GitHub rejected `~/.ssh/id_ed25519` with "Permission denied (publickey)". The key may not be registered, or may need a passphrase the agent hasn't loaded. The user is to push from their own terminal; no credentials were entered or configured here.

### 14:55 · Idea: dual-resolution maps with cheap derivatives
`session b8a64a31` · `live` @ `37f832a` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 14:46 entry above.

- **note** — The user proposed running two maps:
  - a coarse ray map (about 0.2 m) with votes, as discussed;
  - a fine map at half the voxel size (8× the cells), tracking hit membership only, without DDA, as a high-resolution no-go map;
  - a derivative of both that weights the 8 fine voxels inside each coarse voxel.
  Planning would run on the coarse map. The goal is computationally cheap derivatives.
- **test** — Cost of the fine hit-only map, measured on the host (hemisphere sensor, 8 s): point presence at 0.1 m, chunk side 32, versus the coarse vote ray map at 0.2 m, side 16.
  - evidence: fine presence takes 2.7 ms per 230k-point pass at max rate (86 M points/s) and 10.5 ms per pass when paced at 2.3 M/s (the host is slower when idle between passes). Publish 0.2–0.7 ms. 745k hit voxels take 2.1 MB after 8 s paced.
  - evidence: the coarse ray map takes 241 ms per pass (0.96 M rays/s). At max rate, the fine map adds about 1% to the coarse map's work.
  - evidence: at 0.1 m, presence with chunk side 32 is 2.5× faster than side 16 (86 vs 35 M points/s).
- **claim** — Same grid origin, fine size exactly half the coarse, and fine chunk side 32 against coarse side 16 give identical chunk keys and 3.2 m chunks. The 8 sub-voxels of a coarse voxel are then bits in one paired fine chunk, so per-coarse-voxel masks and weights need no lookups. Risk: the fine map has no misses, so it never forgets; coarse free evidence could be used to clear its bits.

### 16:06 · Paired maps prototype
`session b8a64a31` · `live` @ `37f832a` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 14:55 entry above.

- **decision** — The user asked to "Prototype the paired maps: a coarse vote ray map and an aligned half-size hit-only map published together under one generation, with the 8-bit sub-voxel mask derivative and clearing of fine hits from coarse free evidence. Measure the cost."
  - Built `PairedSceneWriter` / `PairedReader` / `PairedSnapshot` / `ClearRule` in `src/live/paired.rs`: coarse `RaySceneWriter<VoteModel,16>` plus fine `PointSceneWriter<Occupied,32>` at half the size and the same origin. Both maps share chunk keys, and one `RwLock` index holds coarse, fine and mask layers with one generation.
  - Crate-internal helpers added: `WorkingChunk::{for_each_observed_mut(local, cell), for_each_observed_local, remove_local}`, `ChunkStore::{chunk, chunk_mut, forget_cells}`, writer `parts_mut`, `votes::was_updated`, and `PublishedSnapshot::new` made `pub(crate)`. New `LiveError::InvalidClearRule`.
- **test** — 56 unit tests pass, 8 of them new:
  - children and parents agree for all 4096 coarse locals;
  - fine keys divided by 2 equal coarse keys for 4 origins × 4 sizes × 4000 points;
  - one generation, and masks equal those rebuilt from the fine snapshot;
  - region bounds are applied at both resolutions;
  - the range filter applies to both maps;
  - clearing works with decay and does nothing without it;
  - `ClearRule` validation;
  - chunk removal, sparse and dense.
  fmt, clippy `-D warnings`, MSRV 1.95.0 and the integration test pass.
- **test** — Unexpected result: in the default release build, paired ingest (204–205 ms per pass) was faster than the coarse map alone (243.5–244.5 ms). With fat LTO and `codegen-units=1`, coarse alone is 207–208 ms and paired 210–211 ms. The ray path is about 15% slower when its generic code is instantiated in the example crate rather than the library. So every earlier ray rate from `bench_scan` (default build) is about 15% pessimistic. `bench_board.sh` now also builds with LTO, and the plan's §7 recommends LTO for applications.
- **test** — Cost of the paired maps with LTO (host, 0.2/0.1 m, hemisphere): ingest +1–2%, publish 7.2 ms vs 1.1 ms (fine copies plus masks about 3 ms, decay plus clearing about 3 ms), about 4% more CPU overall; memory +10% working, +15% published. The first clearing loop tried 8 children for every free coarse voxel and took 9.5–10.4 ms per publish. The current loop walks only existing fine voxels and checks their parents; `paired_study` results are identical.
- **test** — `paired_study` (coarse 0.2 m, hover, box removed after 20 passes).
  - evidence: keep 0.7 with clear (0.25, 1) clears half the box's 2081 fine voxels in 13 passes; 406 are left after 60.
  - evidence: 93 of the leftovers share a coarse voxel with a real surface, so they cannot be cleared through the coarse map. This is a structural limit.
  - evidence: real fine voxels present after the removal average 99.2–99.9%; surfaces hit only intermittently are cleared and re-added.
  - evidence: no decay means no clearing. keep 0.9 clears half in 47–53 passes.
- **decision** — Plan revision (j) adds D14 with cost and behaviour tables; README, benchmark doc and legend (Paired maps, Sub-voxel mask, Clear rule) are updated. Nothing committed.

### 17:01 · Commit of the paired-maps prototype
`session b8a64a31` · `live` @ `37f832a` · [transcript](transcripts/2026-10-21--b8a64a31.jsonl)

Continuation of the 16:06 entry above.

- **note** — The user pushed 37f832a themselves; `live` matched `origin/live` before this commit.
- **decision** — The user asked to "Commit the working tree changes with a sensible message." The paired-maps prototype goes in as one commit on `live`: library, bench mode, study, board script cases, docs, legend and ledger. Not pushed.
- **test** — Before committing: fmt, clippy `-D warnings`, `cargo +1.95.0 check --all-targets`, 56 unit + 1 integration tests and `bash -n scripts/bench_board.sh` pass. All changed files were last modified by 16:06, matching this session's edits. `examples/export_passes.rs` and `examples/vote_order_study.rs` were flagged as changed but have no diff from HEAD.
