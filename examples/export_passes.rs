//! Writes the synthetic warehouse scans as one point cloud per sensor pass, so the test
//! data can be inspected or fed to other tools.
//!
//!     cargo run --release --example export_passes -- --out passes --passes 10
//!
//! For each pass this writes `pass-NNNN.ply` (binary little-endian) with one vertex per
//! return:
//!   x, y, z     the hit, world frame, metres (f32)
//!   sx, sy, sz  the sensor origin when that beam fired, world frame, metres (f32)
//!   t           seconds since the start of the pass (f32)
//! and appends a line to `passes.csv` with the pass start time, return count, sensor pose
//! at the start of the pass and sensor velocity. Passes follow the sensor along the
//! corridor; pass k starts at k / scan_hz seconds.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::io::{BufWriter, Write};
use std::path::PathBuf;

use clap::Parser;

use scene::{Lidar, PERIOD_M, Warehouse};

#[derive(Parser, Debug)]
#[command(about = "Export synthetic LiDAR passes as PLY point clouds")]
struct Args {
    #[arg(long, default_value = "passes")]
    out: PathBuf,
    #[arg(long, default_value_t = 10)]
    passes: usize,
    /// Beams fired per second; beams that hit nothing within range are dropped, so a pass
    /// holds slightly fewer returns.
    #[arg(long, default_value_t = 2.3e6)]
    rate: f64,
    /// Passes per second; each pass is one file.
    #[arg(long, default_value_t = 10.0)]
    scan_hz: f64,
    #[arg(long, default_value_t = 40.0)]
    max_range: f64,
    #[arg(long, default_value_t = -22.5, allow_negative_numbers = true)]
    elevation_min: f64,
    #[arg(long, default_value_t = 22.5, allow_negative_numbers = true)]
    elevation_max: f64,
    /// Sensor speed along the corridor in m/s.
    #[arg(long, default_value_t = 4.0)]
    speed: f64,
    #[arg(long, default_value_t = 7)]
    seed: u64,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();
    std::fs::create_dir_all(&args.out)?;
    let world = Warehouse::new(args.seed);
    let mut lidar = Lidar::for_rate(args.rate, args.scan_hz, args.max_range);
    lidar.elevation_deg = (args.elevation_min, args.elevation_max);
    let period_s = 1.0 / args.scan_hz;
    let speed = args.speed.max(1e-6);
    let per_period = ((PERIOD_M * args.scan_hz / speed).round() as usize).max(2);
    let velocity = [PERIOD_M / per_period as f64 * args.scan_hz, 0.0, 0.0];

    let mut index = BufWriter::new(std::fs::File::create(args.out.join("passes.csv"))?);
    writeln!(
        index,
        "pass,file,start_s,returns,pose_x,pose_y,pose_z,velocity_x,velocity_y,velocity_z"
    )?;
    for pass in 0..args.passes {
        let shift = (pass / per_period) as f64 * PERIOD_M;
        let local = scene::pose(pass % per_period, per_period);
        let scan = scene::cast_scan(
            &world,
            &lidar,
            local,
            velocity,
            period_s,
            args.seed.wrapping_add((pass % per_period) as u64),
        );
        let pose = [scan.pose[0] + shift, scan.pose[1], scan.pose[2]];
        let name = format!("pass-{pass:04}.ply");
        let mut file = BufWriter::new(std::fs::File::create(args.out.join(&name))?);
        write!(
            file,
            "ply\nformat binary_little_endian 1.0\n\
             comment voxelaz synthetic warehouse pass {pass}, start {:.3} s, period {period_s} s\n\
             comment sensor pose at pass start {:.4} {:.4} {:.4}, velocity {:.4} {:.4} {:.4} m/s\n\
             element vertex {}\n\
             property float x\nproperty float y\nproperty float z\n\
             property float sx\nproperty float sy\nproperty float sz\n\
             property float t\nend_header\n",
            pass as f64 * period_s,
            pose[0],
            pose[1],
            pose[2],
            velocity[0],
            velocity[1],
            velocity[2],
            scan.offsets.len()
        )?;
        for (offset, fired) in scan.offsets.iter().zip(&scan.fired) {
            let time = f64::from(*fired) * period_s;
            let values = [
                pose[0] + f64::from(offset[0]),
                pose[1] + f64::from(offset[1]),
                pose[2] + f64::from(offset[2]),
                pose[0] + velocity[0] * time,
                pose[1] + velocity[1] * time,
                pose[2] + velocity[2] * time,
                time,
            ];
            for value in values {
                file.write_all(&(value as f32).to_le_bytes())?;
            }
        }
        file.flush()?;
        writeln!(
            index,
            "{pass},{name},{:.3},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            pass as f64 * period_s,
            scan.offsets.len(),
            pose[0],
            pose[1],
            pose[2],
            velocity[0],
            velocity[1],
            velocity[2]
        )?;
        println!("{name}: {} returns", scan.offsets.len());
    }
    index.flush()
}
