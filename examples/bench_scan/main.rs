//! Sustained live-scene benchmark on synthetic LiDAR scans, for host and board runs.
//!
//! `scripts/bench_board.sh` runs the standard matrix on the target and records the
//! machine state next to the results. One case by hand:
//!
//!     cargo run --release --example bench_scan -- --mode rays --paced --seconds 60
//!
//! Scans are cast once before the timed run (see `scene.rs`). A producer thread shifts
//! them into place and hands them to the writer over a bounded channel, the way the
//! application would. Thread CPU time, RSS and process CPU come from /proc on Linux and
//! print `n/a` elsewhere.

mod scene;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use voxelaz::live::{
    BatchStats, COARSE_SIDE, ClearRule, LogOddsOccupancy, Occupied, PairedSceneConfig,
    PairedSceneWriter, PointCell, PointSceneConfig, PointSceneWriter, PublishStats, RayCellContext,
    RayCellRole, RayObservation, RaySceneConfig, RaySceneWriter, RayUpdateModel, SceneMemoryStats,
    SceneReader, VoteDecay, VoteModel, Votes, VoxelBounds, VoxelGeometry, VoxelKey, visit_ray,
};

use scene::{Lcg, Lidar, PERIOD_M, Scan, Warehouse};

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    /// Point presence from the hits.
    Presence,
    /// Saturating point counts from the hits.
    Count,
    /// Rays from the scan origin to each hit.
    Rays,
    /// Rays from a source that moves during the scan, one source per return.
    RaysMoving,
    /// Vote rays at --voxel plus a half-size hit-only map, published together.
    Paired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Model {
    /// The reference clamped log-odds model (f32 cells).
    LogOdds,
    /// A two-counter custom cell (u16 each), to check a multi-field model.
    Evidence,
    /// Hit and miss votes (`VoteModel`), with the decay from --decay.
    Votes,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Observer {
    None,
    /// Snapshot a region around the sensor, then query the snapshot.
    Snapshot,
    /// Query the reader directly; every lookup takes the index lock.
    Locked,
}

#[derive(Parser, Debug)]
#[command(about = "Sustained live-scene benchmark on synthetic LiDAR scans")]
struct Args {
    #[arg(long, value_enum, default_value_t = Mode::Rays)]
    mode: Mode,
    /// Voxel edge length in metres.
    #[arg(long, default_value_t = 0.2)]
    voxel: f64,
    /// Returns per second the synthetic sensor delivers.
    #[arg(long, default_value_t = 2.3e6)]
    rate: f64,
    #[arg(long, default_value_t = 10.0)]
    scan_hz: f64,
    /// Sensor range in metres; beams that hit nothing closer are dropped.
    #[arg(long, default_value_t = 40.0)]
    max_range: f64,
    /// Lowest beam elevation in degrees; with --elevation-max 90 and 0 here the sensor covers the upper hemisphere.
    #[arg(long, default_value_t = -22.5, allow_negative_numbers = true)]
    elevation_min: f64,
    /// Highest beam elevation in degrees.
    #[arg(long, default_value_t = 22.5, allow_negative_numbers = true)]
    elevation_max: f64,
    /// Feed each scan when the sensor would deliver it, instead of as fast as possible.
    #[arg(long)]
    paced: bool,
    #[arg(long, default_value_t = 20.0)]
    seconds: f64,
    /// Publications per second of wall time; 0 disables publishing.
    #[arg(long, default_value_t = 10.0)]
    publish_hz: f64,
    /// Sensor speed along the corridor in m/s; 0 hovers over one stretch so the map stops growing.
    #[arg(long, default_value_t = 4.0)]
    speed: f64,
    /// Chunk edge in voxels: 8, 16 or 32.
    #[arg(long, default_value_t = 16)]
    chunk_side: usize,
    #[arg(long, value_enum, default_value_t = Model::LogOdds)]
    model: Model,
    /// Decay for --model votes, applied at each publish: none, exp:<keep> or cap:<votes>.
    #[arg(long, default_value = "none", value_parser = parse_decay)]
    decay: VoteDecay,
    /// For --mode paired: clear fine hits once their coarse voxel, decayed at publish, has
    /// at most <hits> and at least <misses> votes ("hits:misses"), or "none".
    #[arg(long, default_value = "0.25:1", value_parser = parse_clear)]
    clear: Clear,
    #[arg(long, value_enum, default_value_t = Observer::None)]
    observer: Observer,
    #[arg(long, default_value_t = 5.0)]
    observer_hz: f64,
    /// Half-width in metres of the box the observer reads around the sensor.
    #[arg(long, default_value_t = 20.0)]
    observer_radius: f64,
    /// Lookups per observer cycle, as a random walk that queries 26 neighbours per step.
    #[arg(long, default_value_t = 1_000_000)]
    observer_lookups: u64,
    #[arg(long, default_value_t = 7)]
    seed: u64,
    /// Name for this case in the result row.
    #[arg(long, default_value = "")]
    label: String,
    /// Print the markdown header for the result rows and exit.
    #[arg(long)]
    table_header: bool,
}

fn main() {
    let args = Args::parse();
    if args.table_header {
        println!(
            "| case | mode | voxel m | paced | input M/s | cells/ray | M cell events/s | writer busy | publish p99 ms | late scans | RSS MB | observer M lookups/s |"
        );
        println!("|---|---|---|---|---|---|---|---|---|---|---|---|");
        return;
    }
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], args.voxel).expect("voxel size");
    match args.chunk_side {
        8 => by_side::<8>(&args, geometry),
        16 => by_side::<16>(&args, geometry),
        32 => by_side::<32>(&args, geometry),
        other => panic!("--chunk-side must be 8, 16 or 32, not {other}"),
    }
}

fn by_side<const SIDE: usize>(args: &Args, geometry: VoxelGeometry) {
    let points = PointSceneConfig::default();
    let rays = RaySceneConfig::default();
    match (args.mode, args.model) {
        (Mode::Presence, _) => {
            let (writer, reader) =
                PointSceneWriter::<Occupied, SIDE>::open(geometry, &points).unwrap();
            bench(args, geometry, writer, Some(reader));
        }
        (Mode::Count, _) => {
            let (writer, reader) = PointSceneWriter::<u16, SIDE>::open(geometry, &points).unwrap();
            bench(args, geometry, writer, Some(reader));
        }
        (Mode::Rays | Mode::RaysMoving, Model::LogOdds) => {
            let (writer, reader) =
                RaySceneWriter::<_, SIDE>::open(geometry, &rays, LogOddsOccupancy::default())
                    .unwrap();
            bench(args, geometry, writer, Some(reader));
        }
        (Mode::Rays | Mode::RaysMoving, Model::Evidence) => {
            let (writer, reader) =
                RaySceneWriter::<_, SIDE>::open(geometry, &rays, EvidenceModel).unwrap();
            bench(args, geometry, writer, Some(reader));
        }
        (Mode::Rays | Mode::RaysMoving, Model::Votes) => {
            let model = VoteModel::new(args.decay).expect("decay");
            let (writer, reader) = RaySceneWriter::<_, SIDE>::open(geometry, &rays, model).unwrap();
            bench(args, geometry, writer, Some(reader));
        }
        (Mode::Paired, _) => {
            assert_eq!(
                SIDE, COARSE_SIDE,
                "paired maps use coarse chunk side {COARSE_SIDE}"
            );
            let config = PairedSceneConfig {
                clear: args.clear.0,
                ..PairedSceneConfig::default()
            };
            let model = VoteModel::new(args.decay).expect("decay");
            let (writer, _) = PairedSceneWriter::open(geometry, &config, model).unwrap();
            let sink = PairedSink {
                writer,
                cleared: 0,
                fine_chunks: 0,
                publishes: 0,
            };
            bench(args, geometry, sink, None::<SceneReader<Votes>>);
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Evidence {
    hits: u16,
    misses: u16,
}

struct EvidenceModel;

impl RayUpdateModel for EvidenceModel {
    type Cell = Evidence;

    fn update(&mut self, cell: &mut Evidence, event: RayCellContext) {
        match event.role {
            RayCellRole::Hit => cell.hits = cell.hits.saturating_add(1),
            RayCellRole::Free => cell.misses = cell.misses.saturating_add(1),
        }
    }
}

/// One scan, shifted into world coordinates. Ray-mode batches with a moving source
/// fill `rays`; every other mode reads `source` and `hits`.
#[derive(Default)]
struct Batch {
    source: [f64; 3],
    hits: Vec<[f64; 3]>,
    rays: Vec<RayObservation>,
}

trait Sink {
    fn ingest(&mut self, batch: &Batch) -> BatchStats;
    fn publish(&mut self) -> PublishStats;
    fn memory(&self) -> SceneMemoryStats;

    fn extra(&self) -> Option<String> {
        None
    }
}

struct PairedSink {
    writer: PairedSceneWriter,
    cleared: u64,
    fine_chunks: u64,
    publishes: u64,
}

impl Sink for PairedSink {
    fn ingest(&mut self, batch: &Batch) -> BatchStats {
        if batch.rays.is_empty() {
            self.writer
                .insert_rays_from_source(batch.source, &batch.hits)
        } else {
            self.writer.insert_rays(&batch.rays)
        }
        .expect("paired batch")
        .coarse
    }

    fn publish(&mut self) -> PublishStats {
        let stats = self.writer.publish().expect("publish");
        self.cleared += stats.fine_cleared;
        self.fine_chunks += stats.fine_chunks;
        self.publishes += 1;
        stats.publish
    }

    fn memory(&self) -> SceneMemoryStats {
        self.writer.memory_stats()
    }

    fn extra(&self) -> Option<String> {
        Some(format!(
            "paired fine_voxel_m={} fine_chunks_per_publish={:.0} fine_voxels_cleared={}",
            self.writer.fine_geometry().voxel_size(),
            self.fine_chunks as f64 / self.publishes.max(1) as f64,
            self.cleared
        ))
    }
}

impl<Cell: PointCell, const SIDE: usize> Sink for PointSceneWriter<Cell, SIDE> {
    fn ingest(&mut self, batch: &Batch) -> BatchStats {
        self.insert_points(&batch.hits).expect("point batch")
    }

    fn publish(&mut self) -> PublishStats {
        PointSceneWriter::publish(self).expect("publish")
    }

    fn memory(&self) -> SceneMemoryStats {
        self.memory_stats()
    }
}

impl<M: RayUpdateModel, const SIDE: usize> Sink for RaySceneWriter<M, SIDE> {
    fn ingest(&mut self, batch: &Batch) -> BatchStats {
        if batch.rays.is_empty() {
            self.insert_rays_from_source(batch.source, &batch.hits)
        } else {
            self.insert_rays(&batch.rays)
        }
        .expect("ray batch")
    }

    fn publish(&mut self) -> PublishStats {
        RaySceneWriter::publish(self).expect("publish")
    }

    fn memory(&self) -> SceneMemoryStats {
        self.memory_stats()
    }
}

/// Which pre-cast scan to play next, and how far along the corridor to shift it.
struct Replay {
    scans_per_period: usize,
    fly: bool,
}

impl Replay {
    fn pick(&self, n: u64) -> (usize, f64) {
        let count = self.scans_per_period as u64;
        if self.fly {
            ((n % count) as usize, (n / count) as f64 * PERIOD_M)
        } else {
            let cycle = (2 * count).saturating_sub(2).max(1);
            let step = n % cycle;
            let index = if step < count { step } else { cycle - step };
            (index as usize, 0.0)
        }
    }
}

fn bench<W: Sink, C: Copy + Send + Sync + 'static>(
    args: &Args,
    geometry: VoxelGeometry,
    mut writer: W,
    reader: Option<SceneReader<C>>,
) {
    let world = Warehouse::new(args.seed);
    let cast_speed = if args.speed > 0.0 { args.speed } else { 4.0 };
    let scans_per_period = ((PERIOD_M * args.scan_hz / cast_speed).round() as usize).max(2);
    let velocity = [PERIOD_M / scans_per_period as f64 * args.scan_hz, 0.0, 0.0];
    let period_s = 1.0 / args.scan_hz;
    let lidar = calibrated_lidar(&world, args, velocity, period_s);
    let cast_started = Instant::now();
    let scans = Arc::new(scene::cast_period(
        &world,
        &lidar,
        scans_per_period,
        velocity,
        period_s,
        args.seed,
    ));
    let cast_s = cast_started.elapsed().as_secs_f64();
    let returns_per_scan =
        scans.iter().map(|scan| scan.offsets.len()).sum::<usize>() as f64 / scans.len() as f64;
    let rays = matches!(args.mode, Mode::Rays | Mode::RaysMoving | Mode::Paired);
    let cells = rays.then(|| cells_per_ray(&geometry, &scans[0]));

    let replay = Replay {
        scans_per_period,
        fly: args.speed > 0.0,
    };
    let stop = Arc::new(AtomicBool::new(false));
    let pose = Arc::new(Mutex::new(scans[0].pose));
    let (batch_tx, batch_rx) = sync_channel::<Batch>(2);
    let (recycle_tx, recycle_rx) = sync_channel::<Batch>(4);
    let producer = {
        let scans = Arc::clone(&scans);
        let stop = Arc::clone(&stop);
        let moving = args.mode == Mode::RaysMoving;
        std::thread::spawn(move || {
            produce(
                &scans, &replay, velocity, period_s, moving, batch_tx, recycle_rx, &stop,
            )
        })
    };
    let observer = (args.observer != Observer::None).then(|| {
        let reader = reader.clone().expect("--observer needs a single scene");
        let stop = Arc::clone(&stop);
        let pose = Arc::clone(&pose);
        let plan = ObserverPlan {
            mode: args.observer,
            hz: args.observer_hz,
            radius_m: args.observer_radius,
            lookups: args.observer_lookups,
            seed: args.seed,
        };
        std::thread::spawn(move || observe(reader, plan, &pose, &stop))
    });

    let run = drive(args, &mut writer, &batch_rx, &recycle_tx, &pose, period_s);
    stop.store(true, Ordering::Relaxed);
    drop(batch_rx);
    let producer_cpu = producer.join().expect("producer");
    let observed = observer.map(|handle| handle.join().expect("observer"));
    let memory = writer.memory();
    if let Some(extra) = writer.extra() {
        println!("{extra}");
    }
    report(
        args,
        &lidar,
        scans_per_period,
        cast_s,
        returns_per_scan,
        cells,
        &run,
        producer_cpu,
        observed.as_ref(),
        &memory,
    );
}

/// The --clear setting; a newtype so clap does not treat it as an optional argument.
#[derive(Clone, Copy, Debug)]
struct Clear(Option<ClearRule>);

/// Parses `none` or `<hits>:<misses>`.
fn parse_clear(text: &str) -> Result<Clear, String> {
    if text == "none" {
        return Ok(Clear(None));
    }
    let (hits, misses) = text
        .split_once(':')
        .ok_or_else(|| format!("clear must be none or <hits>:<misses>, not {text}"))?;
    let parse = |value: &str| {
        value
            .parse::<f64>()
            .map_err(|_| format!("bad number {value}"))
    };
    ClearRule::new(parse(hits)?, parse(misses)?)
        .map(|rule| Clear(Some(rule)))
        .map_err(|error| error.to_string())
}

/// Parses `none`, `exp:0.9` or `cap:1000`.
fn parse_decay(text: &str) -> Result<VoteDecay, String> {
    let decay = match text.split_once(':') {
        None if text == "none" => VoteDecay::None,
        Some(("exp", keep)) => VoteDecay::Exponential {
            keep: keep.parse().map_err(|_| format!("bad keep {keep}"))?,
        },
        Some(("cap", cap)) => VoteDecay::CapHalving {
            cap_votes: cap.parse().map_err(|_| format!("bad cap {cap}"))?,
        },
        _ => {
            return Err(format!(
                "decay must be none, exp:<keep> or cap:<votes>, not {text}"
            ));
        }
    };
    VoteModel::new(decay)
        .map(|_| decay)
        .map_err(|error| error.to_string())
}

/// Picks azimuth steps so the returns, not the fired beams, reach `--rate`.
fn calibrated_lidar(world: &Warehouse, args: &Args, velocity: [f64; 3], period_s: f64) -> Lidar {
    let mut lidar = Lidar::for_rate(args.rate, args.scan_hz, args.max_range);
    lidar.elevation_deg = (args.elevation_min, args.elevation_max);
    let probe = scene::cast_scan(world, &lidar, scene::pose(0, 2), velocity, period_s, 1);
    let kept = probe.offsets.len() as f64 / lidar.fired_per_scan() as f64;
    if kept > 0.0 {
        let fired = args.rate / args.scan_hz / kept;
        lidar.azimuth_steps = (fired / lidar.channels as f64).round().max(1.0) as usize;
    }
    lidar
}

struct CellsPerRay {
    mean: f64,
    p95: u64,
}

fn cells_per_ray(geometry: &VoxelGeometry, scan: &Scan) -> CellsPerRay {
    let mut counts = Vec::new();
    for offset in scan.offsets.iter().step_by(31) {
        let hit = [
            scan.pose[0] + f64::from(offset[0]),
            scan.pose[1] + f64::from(offset[1]),
            scan.pose[2] + f64::from(offset[2]),
        ];
        let mut cells = 0u64;
        visit_ray(geometry, scan.pose, hit, None, 1 << 24, |_| cells += 1).expect("sample ray");
        counts.push(cells);
    }
    counts.sort_unstable();
    CellsPerRay {
        mean: counts.iter().sum::<u64>() as f64 / counts.len().max(1) as f64,
        p95: percentile(&counts, 0.95).unwrap_or(0),
    }
}

#[allow(clippy::too_many_arguments)]
fn produce(
    scans: &[Scan],
    replay: &Replay,
    velocity: [f64; 3],
    period_s: f64,
    moving: bool,
    batches: SyncSender<Batch>,
    recycled: Receiver<Batch>,
    stop: &AtomicBool,
) -> Option<f64> {
    let cpu_started = thread_cpu_s();
    let mut n = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let (index, shift) = replay.pick(n);
        let scan = &scans[index];
        let mut batch = recycled.try_recv().unwrap_or_default();
        batch.hits.clear();
        batch.rays.clear();
        let origin = [scan.pose[0] + shift, scan.pose[1], scan.pose[2]];
        batch.source = origin;
        for (offset, fired) in scan.offsets.iter().zip(&scan.fired) {
            let hit = [
                origin[0] + f64::from(offset[0]),
                origin[1] + f64::from(offset[1]),
                origin[2] + f64::from(offset[2]),
            ];
            if moving {
                let time = f64::from(*fired) * period_s;
                let source = [
                    origin[0] + velocity[0] * time,
                    origin[1] + velocity[1] * time,
                    origin[2] + velocity[2] * time,
                ];
                batch.rays.push(RayObservation { source, hit });
            } else {
                batch.hits.push(hit);
            }
        }
        if batches.send(batch).is_err() {
            break;
        }
        n += 1;
    }
    Some(thread_cpu_s()? - cpu_started?)
}

#[derive(Default)]
struct Run {
    wall_s: f64,
    scans: u64,
    stats: BatchStats,
    ingest_ms: Vec<f64>,
    publish_ms: Vec<f64>,
    published_chunks: u64,
    late_scans: u64,
    max_lag_ms: f64,
    writer_cpu_s: Option<f64>,
    process_cpu_s: Option<f64>,
}

fn drive<W: Sink>(
    args: &Args,
    writer: &mut W,
    batches: &Receiver<Batch>,
    recycle: &SyncSender<Batch>,
    pose: &Mutex<[f64; 3]>,
    period_s: f64,
) -> Run {
    let total_scans = (args.seconds * args.scan_hz).round().max(1.0) as u64;
    let publish_every =
        (args.publish_hz > 0.0).then(|| Duration::from_secs_f64(1.0 / args.publish_hz));
    let mut run = Run::default();
    let writer_cpu = thread_cpu_s();
    let process_cpu = process_cpu_s();
    let started = Instant::now();
    let mut next_publish = publish_every.map(|every| started + every);
    loop {
        let finished = if args.paced {
            // A writer that cannot keep up stops at the wall-clock limit too, with the
            // backlog reported as late scans and lag.
            run.scans >= total_scans || started.elapsed().as_secs_f64() >= args.seconds + period_s
        } else {
            started.elapsed().as_secs_f64() >= args.seconds
        };
        if finished {
            break;
        }
        let Ok(batch) = batches.recv() else {
            break;
        };
        if args.paced {
            let due = started + Duration::from_secs_f64(run.scans as f64 * period_s);
            let now = Instant::now();
            if now < due {
                std::thread::sleep(due - now);
            } else {
                // A scan counts as late once the writer is a whole scan period behind.
                let lag_ms = (now - due).as_secs_f64() * 1e3;
                if lag_ms > period_s * 1e3 {
                    run.late_scans += 1;
                }
                run.max_lag_ms = run.max_lag_ms.max(lag_ms);
            }
        }
        let ingest_started = Instant::now();
        let stats = writer.ingest(&batch);
        run.ingest_ms
            .push(ingest_started.elapsed().as_secs_f64() * 1e3);
        accumulate(&mut run.stats, &stats);
        *pose.lock().unwrap() = batch.source;
        let _ = recycle.try_send(batch);
        if let (Some(every), Some(next)) = (publish_every, next_publish)
            && Instant::now() >= next
        {
            let published = writer.publish();
            run.publish_ms.push(published.duration_ns as f64 / 1e6);
            run.published_chunks += published.chunks_published;
            let now = Instant::now();
            next_publish = Some(if now > next + every {
                now + every
            } else {
                next + every
            });
        }
        run.scans += 1;
    }
    run.wall_s = started.elapsed().as_secs_f64();
    run.writer_cpu_s = thread_cpu_s()
        .zip(writer_cpu)
        .map(|(end, start)| end - start);
    run.process_cpu_s = process_cpu_s()
        .zip(process_cpu)
        .map(|(end, start)| end - start);
    run
}

fn accumulate(total: &mut BatchStats, batch: &BatchStats) {
    total.input += batch.input;
    total.accepted += batch.accepted;
    total.filtered_by_range += batch.filtered_by_range;
    total.invalid += batch.invalid;
    total.distance_overflow += batch.distance_overflow;
    total.capacity_rejected += batch.capacity_rejected;
    total.hits_applied += batch.hits_applied;
    total.free_cell_updates += batch.free_cell_updates;
    total.newly_observed_cells += batch.newly_observed_cells;
    total.dirty_chunks += batch.dirty_chunks;
}

struct ObserverPlan {
    mode: Observer,
    hz: f64,
    radius_m: f64,
    lookups: u64,
    seed: u64,
}

struct Observed {
    cycles: u64,
    snapshot_ms: Vec<f64>,
    age_ms: Vec<f64>,
    lookups: u64,
    lookup_s: f64,
    known: u64,
    cpu_s: Option<f64>,
}

const NEIGHBOURS: usize = 26;

fn observe<C: Copy + Send + Sync + 'static>(
    reader: SceneReader<C>,
    plan: ObserverPlan,
    pose: &Mutex<[f64; 3]>,
    stop: &AtomicBool,
) -> Observed {
    let geometry = reader.geometry();
    let cpu_started = thread_cpu_s();
    let mut rng = Lcg::new(plan.seed ^ 0x0b5e_47e5);
    let mut out = Observed {
        cycles: 0,
        snapshot_ms: Vec::new(),
        age_ms: Vec::new(),
        lookups: 0,
        lookup_s: 0.0,
        known: 0,
        cpu_s: None,
    };
    let cycle = Duration::from_secs_f64(1.0 / plan.hz.max(0.01));
    while !stop.load(Ordering::Relaxed) {
        let cycle_started = Instant::now();
        let center = *pose.lock().unwrap();
        let reach = plan.radius_m;
        let low = geometry.voxel_key([center[0] - reach, center[1] - reach, center[2] - reach]);
        let high = geometry.voxel_key([center[0] + reach, center[1] + reach, center[2] + reach]);
        let bounds = VoxelBounds::try_new(low.unwrap(), high.unwrap()).unwrap();
        let steps = plan.lookups / NEIGHBOURS as u64;
        let walk_started;
        match plan.mode {
            Observer::Snapshot => {
                let snapshot_started = Instant::now();
                let snapshot = reader.snapshot_region(bounds).unwrap();
                out.snapshot_ms
                    .push(snapshot_started.elapsed().as_secs_f64() * 1e3);
                if let Some(age) = snapshot.age() {
                    out.age_ms.push(age.as_secs_f64() * 1e3);
                }
                walk_started = Instant::now();
                out.known += walk(&bounds, steps, &mut rng, |key| snapshot.get(key).is_some());
            }
            Observer::Locked => {
                walk_started = Instant::now();
                out.known += walk(&bounds, steps, &mut rng, |key| {
                    reader.get_voxel_key(key).is_some()
                });
            }
            Observer::None => unreachable!(),
        }
        out.lookup_s += walk_started.elapsed().as_secs_f64();
        out.lookups += steps * NEIGHBOURS as u64;
        out.cycles += 1;
        if let Some(rest) = cycle.checked_sub(cycle_started.elapsed()) {
            std::thread::sleep(rest);
        }
    }
    out.cpu_s = thread_cpu_s()
        .zip(cpu_started)
        .map(|(end, start)| end - start);
    out
}

/// A random walk through `bounds` that queries all 26 neighbours at each step, as an
/// A* expansion would. Returns how many queried voxels were known.
fn walk(
    bounds: &VoxelBounds,
    steps: u64,
    rng: &mut Lcg,
    mut known: impl FnMut(VoxelKey) -> bool,
) -> u64 {
    let pick = |rng: &mut Lcg, lo: i64, hi: i64| lo + (rng.unit() * (hi - lo + 1) as f64) as i64;
    let mut at = VoxelKey {
        x: pick(rng, bounds.min.x, bounds.max.x),
        y: pick(rng, bounds.min.y, bounds.max.y),
        z: pick(rng, bounds.min.z, bounds.max.z),
    };
    let mut count = 0u64;
    for _ in 0..steps {
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let key = VoxelKey {
                        x: at.x + dx,
                        y: at.y + dy,
                        z: at.z + dz,
                    };
                    count += u64::from(known(key));
                }
            }
        }
        let step =
            |rng: &mut Lcg, value: i64, lo: i64, hi: i64| (value + pick(rng, -1, 1)).clamp(lo, hi);
        at = VoxelKey {
            x: step(rng, at.x, bounds.min.x, bounds.max.x),
            y: step(rng, at.y, bounds.min.y, bounds.max.y),
            z: step(rng, at.z, bounds.min.z, bounds.max.z),
        };
    }
    count
}

#[allow(clippy::too_many_arguments)]
fn report(
    args: &Args,
    lidar: &Lidar,
    scans_per_period: usize,
    cast_s: f64,
    returns_per_scan: f64,
    cells: Option<CellsPerRay>,
    run: &Run,
    producer_cpu_s: Option<f64>,
    observed: Option<&Observed>,
    memory: &SceneMemoryStats,
) {
    let mut ingest = run.ingest_ms.clone();
    let mut publish = run.publish_ms.clone();
    sort_f64(&mut ingest);
    sort_f64(&mut publish);
    let events = run.stats.hits_applied + run.stats.free_cell_updates;
    let input_rate = run.stats.input as f64 / run.wall_s;
    let busy = run.writer_cpu_s.map(|cpu| cpu / run.wall_s);
    let rss_mb = status_kb("VmRSS:").map(|kb| kb / 1024.0);
    let label = if args.label.is_empty() {
        format!("{:?}", args.mode).to_lowercase()
    } else {
        args.label.clone()
    };
    let opt = |value: Option<f64>, digits: usize| match value {
        Some(value) => format!("{value:.digits$}"),
        None => "n/a".to_string(),
    };

    println!("# bench_scan {label}");
    println!("args={args:?}");
    println!(
        "sensor channels={} elevation_deg={:?} azimuth_steps={} fired_per_scan={} returns_per_scan={:.0} scans_cast={} cast_s={:.2}",
        lidar.channels,
        lidar.elevation_deg,
        lidar.azimuth_steps,
        lidar.fired_per_scan(),
        returns_per_scan,
        scans_per_period,
        cast_s
    );
    if let Some(cells) = &cells {
        println!(
            "cells_per_ray_mean={:.1} cells_per_ray_p95={}",
            cells.mean, cells.p95
        );
    }
    println!(
        "wall_s={:.2} scans={} input={} accepted={} input_per_s={:.0} accepted_per_s={:.0}",
        run.wall_s,
        run.scans,
        run.stats.input,
        run.stats.accepted,
        input_rate,
        run.stats.accepted as f64 / run.wall_s
    );
    println!(
        "cell_events={} cell_events_per_s={:.0} free_events={} hit_events={} newly_observed={}",
        events,
        events as f64 / run.wall_s,
        run.stats.free_cell_updates,
        run.stats.hits_applied,
        run.stats.newly_observed_cells
    );
    println!(
        "scan_ingest_ms p50={} p99={} max={} late_scans={} max_lag_ms={:.1}",
        opt(percentile(&ingest, 0.5), 2),
        opt(percentile(&ingest, 0.99), 2),
        opt(ingest.last().copied(), 2),
        run.late_scans,
        run.max_lag_ms
    );
    println!(
        "publish count={} chunks_per_publish={:.0} ms p50={} p95={} p99={} max={}",
        publish.len(),
        run.published_chunks as f64 / publish.len().max(1) as f64,
        opt(percentile(&publish, 0.5), 3),
        opt(percentile(&publish, 0.95), 3),
        opt(percentile(&publish, 0.99), 3),
        opt(publish.last().copied(), 3)
    );
    println!(
        "cpu_s writer={} producer={} observer={} process={} writer_busy={} process_core_equivalents={}",
        opt(run.writer_cpu_s, 2),
        opt(producer_cpu_s, 2),
        opt(observed.and_then(|observed| observed.cpu_s), 2),
        opt(run.process_cpu_s, 2),
        opt(busy, 3),
        opt(run.process_cpu_s.map(|cpu| cpu / run.wall_s), 3)
    );
    println!(
        "memory rss_mb={} hwm_mb={} working_chunks={} observed_cells={} est_working_mb={:.1} est_published_mb={:.1} generation={}",
        opt(rss_mb, 1),
        opt(status_kb("VmHWM:").map(|kb| kb / 1024.0), 1),
        memory.working_chunks,
        memory.observed_cells,
        memory.estimated_working_bytes as f64 / 1e6,
        memory.estimated_published_bytes as f64 / 1e6,
        memory.generation
    );
    let lookup_rate =
        observed.map(|observed| observed.lookups as f64 / observed.lookup_s.max(1e-9));
    if let Some(observed) = observed {
        let mut snapshot = observed.snapshot_ms.clone();
        let mut age = observed.age_ms.clone();
        sort_f64(&mut snapshot);
        sort_f64(&mut age);
        println!(
            "observer mode={:?} cycles={} lookups={} lookups_per_s={:.0} known_fraction={:.3} snapshot_ms p50={} p99={} age_ms p50={} max={}",
            args.observer,
            observed.cycles,
            observed.lookups,
            lookup_rate.unwrap_or(0.0),
            observed.known as f64 / observed.lookups.max(1) as f64,
            opt(percentile(&snapshot, 0.5), 2),
            opt(percentile(&snapshot, 0.99), 2),
            opt(percentile(&age, 0.5), 1),
            opt(age.last().copied(), 1)
        );
    }
    println!(
        "row: | {label} | {:?} | {} | {} | {:.2} | {} | {:.1} | {} | {} | {} | {} | {} |",
        args.mode,
        args.voxel,
        args.paced,
        input_rate / 1e6,
        cells.map_or("-".to_string(), |cells| format!("{:.0}", cells.mean)),
        events as f64 / run.wall_s / 1e6,
        opt(busy, 2),
        opt(percentile(&publish, 0.99), 2),
        run.late_scans,
        opt(rss_mb, 0),
        lookup_rate.map_or("-".to_string(), |rate| format!("{:.1}", rate / 1e6)),
    );
}

fn sort_f64(values: &mut [f64]) {
    values.sort_unstable_by(f64::total_cmp);
}

fn percentile<T: Copy>(sorted: &[T], quantile: f64) -> Option<T> {
    if sorted.is_empty() {
        return None;
    }
    let index = ((sorted.len() - 1) as f64 * quantile).round() as usize;
    Some(sorted[index])
}

/// CPU time of the calling thread, from the scheduler's nanosecond counter.
fn thread_cpu_s() -> Option<f64> {
    let text = std::fs::read_to_string("/proc/thread-self/schedstat").ok()?;
    let nanos: f64 = text.split_whitespace().next()?.parse().ok()?;
    Some(nanos / 1e9)
}

/// User plus system CPU time of the whole process. /proc reports it in USER_HZ ticks,
/// which Linux fixes at 100 per second.
fn process_cpu_s() -> Option<f64> {
    let text = std::fs::read_to_string("/proc/self/stat").ok()?;
    let fields: Vec<&str> = text
        .get(text.rfind(')')? + 2..)?
        .split_whitespace()
        .collect();
    let user: f64 = fields.get(11)?.parse().ok()?;
    let system: f64 = fields.get(12)?.parse().ok()?;
    Some((user + system) / 100.0)
}

fn status_kb(key: &str) -> Option<f64> {
    let text = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = text.lines().find(|line| line.starts_with(key))?;
    line.split_whitespace().nth(1)?.parse().ok()
}
