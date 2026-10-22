//! Renders the synthetic warehouse used by `bench_scan` and the studies, as PPM images.
//! The warehouse is analytic geometry (planes and boxes) that the simulated sensor is
//! ray-cast against to produce test point clouds, one 100 ms pass at a time. The map
//! images show what the live scene builds from those points with `VoteModel`.
//!
//!     cargo run --release --example scene_render -- --out scene-images --voxel 0.2 \
//!         --blocked-share 0.10 --min-hits 1
//!
//! Writes:
//!   scan-top.ppm               one band pass (±22.5°) from above, coloured by range
//!                              (purple near, yellow at 40 m), ceiling returns hidden
//!   scan-perspective.ppm       three band passes from behind the sensor, ceiling hidden
//!   hemisphere-perspective.ppm one upper-hemisphere pass (0° to 90°), ceiling shown
//!   map-voxels.ppm             blocked voxels after ten passes as cubes coloured by height
//!   map-slice-1m.ppm           the voxel layer containing z = 1.0 m
//!   map-slice-floor.ppm        the two voxel layers either side of the floor
//!   share-voxels.ppm           every voxel with at least one hit, coloured by hit share
//!   share-slice-1m.ppm         the 1 m layer coloured by hit share
//!   share-slice-floor.ppm      the floor layers coloured by hit share
//!
//! Blocked follows `--blocked-share` and `--min-hits` (a `BlockedRule`). Map slices: dark
//! is unknown, grey is free, red is blocked, yellow has hits but is not blocked. Hit-share
//! images map 0 to 100% onto the colour bar at the bottom left (purple to yellow); grey
//! is observed with no hits. The ceiling layer is hidden in the 3D views, and the sensor
//! path is cyan.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::path::{Path, PathBuf};

use clap::Parser;
use voxelaz::live::{
    BlockedRule, RayObservation, RaySceneConfig, RaySceneWriter, VoteModel, Votes, VoxelGeometry,
    VoxelKey,
};

use scene::{Lidar, PERIOD_M, Scan, Warehouse};

#[derive(Parser, Debug)]
#[command(about = "Render the synthetic warehouse and the vote map built from it")]
struct Args {
    #[arg(long, default_value = "scene-images")]
    out: PathBuf,
    /// Voxel edge length in metres for the map images.
    #[arg(long, default_value_t = 0.2)]
    voxel: f64,
    /// A voxel is blocked when hits are at least this fraction of its votes (0 to 1).
    #[arg(long, default_value_t = 0.10)]
    blocked_share: f64,
    /// A blocked voxel also needs at least this many hits (fractions allowed).
    #[arg(long, default_value_t = 1.0)]
    min_hits: f64,
}

type Rgb = [u8; 3];

const UNKNOWN: Rgb = [22, 26, 36];
const FREE: Rgb = [170, 178, 190];
const BLOCKED: Rgb = [230, 80, 50];
const ERODED: Rgb = [250, 200, 60];
const SENSOR: Rgb = [80, 200, 255];
const OUTLINE: Rgb = [235, 235, 235];

struct Canvas {
    width: usize,
    height: usize,
    pixels: Vec<Rgb>,
    depth: Vec<f64>,
}

impl Canvas {
    fn new(width: usize, height: usize, background: Rgb) -> Self {
        Self {
            width,
            height,
            pixels: vec![background; width * height],
            depth: vec![f64::INFINITY; width * height],
        }
    }

    fn set(&mut self, x: i64, y: i64, color: Rgb) {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            self.pixels[y as usize * self.width + x as usize] = color;
        }
    }

    /// Draws a size×size dot if it is nearer than what the pixel already holds.
    fn splat(&mut self, x: i64, y: i64, depth: f64, size: i64, color: Rgb) {
        for dy in 0..size {
            for dx in 0..size {
                let (px, py) = (x + dx, y + dy);
                if px < 0 || py < 0 || px as usize >= self.width || py as usize >= self.height {
                    continue;
                }
                let index = py as usize * self.width + px as usize;
                if depth < self.depth[index] {
                    self.depth[index] = depth;
                    self.pixels[index] = color;
                }
            }
        }
    }

    fn line(&mut self, from: (f64, f64), to: (f64, f64), color: Rgb) {
        let steps = (to.0 - from.0)
            .abs()
            .max((to.1 - from.1).abs())
            .ceil()
            .max(1.0) as usize;
        for step in 0..=steps {
            let t = step as f64 / steps as f64;
            let x = from.0 + (to.0 - from.0) * t;
            let y = from.1 + (to.1 - from.1) * t;
            self.set(x.round() as i64, y.round() as i64, color);
        }
    }

    fn disk(&mut self, x: f64, y: f64, radius: f64, color: Rgb) {
        let r = radius.ceil() as i64;
        for dy in -r..=r {
            for dx in -r..=r {
                if ((dx * dx + dy * dy) as f64) <= radius * radius {
                    self.set(x.round() as i64 + dx, y.round() as i64 + dy, color);
                }
            }
        }
    }

    /// Fills a convex quad at one depth, darkening a one-pixel border so cubes read as cubes.
    fn fill_quad(&mut self, corners: [(f64, f64); 4], depth: f64, color: Rgb) {
        let min_x = corners
            .iter()
            .map(|c| c.0)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as usize;
        let max_x = (corners
            .iter()
            .map(|c| c.0)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil() as i64)
            .min(self.width as i64 - 1);
        let min_y = corners
            .iter()
            .map(|c| c.1)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as usize;
        let max_y = (corners
            .iter()
            .map(|c| c.1)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil() as i64)
            .min(self.height as i64 - 1);
        if max_x < 0 || max_y < 0 {
            return;
        }
        let edge = dim(color, 0.55);
        for y in min_y..=max_y as usize {
            for x in min_x..=max_x as usize {
                let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
                let mut sign = 0.0f64;
                let mut nearest = f64::INFINITY;
                let mut inside = true;
                for side in 0..4 {
                    let (a, b) = (corners[side], corners[(side + 1) % 4]);
                    let length = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt().max(1e-9);
                    let value = (b.0 - a.0) * (py - a.1) - (b.1 - a.1) * (px - a.0);
                    if value != 0.0 {
                        if sign == 0.0 {
                            sign = value.signum();
                        } else if value.signum() != sign {
                            inside = false;
                            break;
                        }
                    }
                    nearest = nearest.min(value.abs() / length);
                }
                let index = y * self.width + x;
                if inside && depth < self.depth[index] {
                    self.depth[index] = depth;
                    self.pixels[index] = if nearest < 1.0 { edge } else { color };
                }
            }
        }
    }

    /// 3×5 digits and '%', drawn `scale` pixels per dot.
    fn text(&mut self, x: i64, y: i64, scale: i64, text: &str, color: Rgb) {
        const GLYPHS: [(char, [u8; 5]); 11] = [
            ('0', [7, 5, 5, 5, 7]),
            ('1', [2, 6, 2, 2, 7]),
            ('2', [7, 1, 7, 4, 7]),
            ('3', [7, 1, 7, 1, 7]),
            ('4', [5, 5, 7, 1, 1]),
            ('5', [7, 4, 7, 1, 7]),
            ('6', [7, 4, 7, 5, 7]),
            ('7', [7, 1, 1, 1, 1]),
            ('8', [7, 5, 7, 5, 7]),
            ('9', [7, 5, 7, 1, 7]),
            ('%', [5, 1, 2, 4, 5]),
        ];
        let mut cursor = x;
        for character in text.chars() {
            if let Some((_, rows)) = GLYPHS.iter().find(|(glyph, _)| *glyph == character) {
                for (row, bits) in rows.iter().enumerate() {
                    for column in 0..3 {
                        if bits & (4 >> column) != 0 {
                            for dy in 0..scale {
                                for dx in 0..scale {
                                    self.set(
                                        cursor + column * scale + dx,
                                        y + row as i64 * scale + dy,
                                        color,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            cursor += 4 * scale;
        }
    }

    /// A 0–100% colour bar with labels in the bottom-left corner.
    fn colour_bar(&mut self) {
        let (left, width, height) = (24i64, 320i64, 14i64);
        let top = self.height as i64 - 52;
        for y in top - 10..self.height as i64 {
            for x in 0..left + width + 24 {
                self.set(x, y, UNKNOWN);
            }
        }
        for dx in 0..width {
            let color = ramp(dx as f64 / (width - 1) as f64);
            for dy in 0..height {
                self.set(left + dx, top + dy, color);
            }
        }
        for (fraction, label) in [(0.0, "0%"), (0.5, "50%"), (1.0, "100%")] {
            let x = left + (fraction * (width - 1) as f64) as i64;
            let shift = (label.len() as i64 * 8) / 2;
            self.text(x - shift, top + height + 6, 2, label, OUTLINE);
        }
    }

    fn save(&self, path: &Path) {
        let mut bytes = format!("P6\n{} {}\n255\n", self.width, self.height).into_bytes();
        for pixel in &self.pixels {
            bytes.extend_from_slice(pixel);
        }
        std::fs::write(path, bytes).expect("write image");
        println!("wrote {}", path.display());
    }
}

/// A pinhole camera looking from `eye` at `target`, with z up.
struct Camera {
    eye: [f64; 3],
    forward: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    width: usize,
    height: usize,
    focal: f64,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let length = dot(v, v).sqrt();
    [v[0] / length, v[1] / length, v[2] / length]
}

impl Camera {
    /// Behind and above the sensor at `pose`, looking ahead along +x.
    fn behind(pose: [f64; 3]) -> Self {
        let eye = [pose[0] - 14.0, pose[1] - 11.0, 6.0];
        let target = [pose[0] + 6.0, pose[1] + 1.0, 1.5];
        let forward = unit([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
        let right = unit(cross(forward, [0.0, 0.0, 1.0]));
        Self {
            eye,
            forward,
            right,
            up: cross(right, forward),
            width: 1600,
            height: 900,
            focal: 900.0,
        }
    }

    /// Pixel position and depth, or `None` for points behind or too near the camera.
    fn project(&self, p: [f64; 3]) -> Option<(f64, f64, f64)> {
        let d = [p[0] - self.eye[0], p[1] - self.eye[1], p[2] - self.eye[2]];
        let z = dot(d, self.forward);
        (z > 0.5).then(|| {
            (
                self.width as f64 / 2.0 + self.focal * dot(d, self.right) / z,
                self.height as f64 / 2.0 - self.focal * dot(d, self.up) / z,
                z,
            )
        })
    }
}

/// Viridis-like ramp, t in [0, 1].
fn ramp(t: f64) -> Rgb {
    const STOPS: [[f64; 3]; 5] = [
        [68.0, 1.0, 84.0],
        [59.0, 82.0, 139.0],
        [33.0, 145.0, 140.0],
        [94.0, 201.0, 98.0],
        [253.0, 231.0, 37.0],
    ];
    let t = t.clamp(0.0, 1.0) * (STOPS.len() - 1) as f64;
    let index = (t.floor() as usize).min(STOPS.len() - 2);
    let frac = t - index as f64;
    let mix =
        |axis: usize| STOPS[index][axis] + (STOPS[index + 1][axis] - STOPS[index][axis]) * frac;
    [mix(0) as u8, mix(1) as u8, mix(2) as u8]
}

fn dim(color: Rgb, factor: f64) -> Rgb {
    color.map(|channel| (channel as f64 * factor).clamp(0.0, 255.0) as u8)
}

/// World points of a scan.
fn returns(scan: &Scan) -> Vec<[f64; 3]> {
    scan.offsets
        .iter()
        .map(|offset| {
            [
                scan.pose[0] + f64::from(offset[0]),
                scan.pose[1] + f64::from(offset[1]),
                scan.pose[2] + f64::from(offset[2]),
            ]
        })
        .collect()
}

/// Box outlines and corridor walls in plan view, copied along x across [x0, x1].
fn plan_outlines(
    canvas: &mut Canvas,
    world: &Warehouse,
    to_px: impl Fn(f64, f64) -> (f64, f64),
    x0: f64,
    x1: f64,
) {
    let (half_width, _) = world.extent();
    canvas.line(to_px(x0, half_width), to_px(x1, half_width), OUTLINE);
    canvas.line(to_px(x0, -half_width), to_px(x1, -half_width), OUTLINE);
    let first = (x0 / PERIOD_M).floor() as i64 - 1;
    let last = (x1 / PERIOD_M).ceil() as i64;
    for k in first..=last {
        let shift = k as f64 * PERIOD_M;
        for (min, max) in world.boxes() {
            let corners = [
                (min[0] + shift, min[1]),
                (max[0] + shift, min[1]),
                (max[0] + shift, max[1]),
                (min[0] + shift, max[1]),
            ];
            for side in 0..4 {
                let (a, b) = (corners[side], corners[(side + 1) % 4]);
                canvas.line(to_px(a.0, a.1), to_px(b.0, b.1), OUTLINE);
            }
        }
    }
}

fn render_top(world: &Warehouse, scan: &Scan, path: &Path) {
    let (x0, x1, y0, y1, scale) = (scan.pose[0] - 42.0, scan.pose[0] + 42.0, -16.0, 16.0, 20.0);
    let mut canvas = Canvas::new(
        ((x1 - x0) * scale) as usize,
        ((y1 - y0) * scale) as usize,
        UNKNOWN,
    );
    let to_px = |x: f64, y: f64| ((x - x0) * scale, (y1 - y) * scale);
    let (_, ceiling) = world.extent();
    for point in returns(scan) {
        if point[2] > ceiling - 0.1 {
            continue;
        }
        let (px, py) = to_px(point[0], point[1]);
        let range = ((point[0] - scan.pose[0]).powi(2)
            + (point[1] - scan.pose[1]).powi(2)
            + (point[2] - scan.pose[2]).powi(2))
        .sqrt();
        canvas.splat(
            px as i64,
            py as i64,
            -point[2],
            2,
            ramp(0.2 + 0.8 * range / 40.0),
        );
    }
    plan_outlines(&mut canvas, world, to_px, x0, x1);
    let (px, py) = to_px(scan.pose[0], scan.pose[1]);
    canvas.disk(px, py, 6.0, SENSOR);
    canvas.save(path);
}

fn render_perspective(world: &Warehouse, scans: &[Scan], hide_ceiling: bool, path: &Path) {
    let pose = scans[0].pose;
    let (_, ceiling) = world.extent();
    let camera = Camera::behind(pose);
    let project = |p: [f64; 3]| camera.project(p);
    let mut canvas = Canvas::new(camera.width, camera.height, UNKNOWN);
    for scan in scans {
        for point in returns(scan) {
            if hide_ceiling && point[2] > ceiling - 0.1 {
                continue;
            }
            if let Some((u, v, depth)) = project(point) {
                let fog = (1.0 - (depth - 10.0) / 70.0).clamp(0.35, 1.0);
                canvas.splat(u as i64, v as i64, depth, 2, dim(ramp(point[2] / 8.0), fog));
            }
        }
    }
    let first = ((pose[0] - 40.0) / PERIOD_M).floor() as i64;
    let last = ((pose[0] + 60.0) / PERIOD_M).ceil() as i64;
    for k in first..=last {
        let shift = k as f64 * PERIOD_M;
        for (min, max) in world.boxes() {
            let corner = |i: usize| {
                [
                    if i & 1 == 0 { min[0] } else { max[0] } + shift,
                    if i & 2 == 0 { min[1] } else { max[1] },
                    if i & 4 == 0 { min[2] } else { max[2] },
                ]
            };
            for a in 0..8usize {
                for bit in [1usize, 2, 4] {
                    let b = a | bit;
                    if b == a {
                        continue;
                    }
                    if let (Some(p), Some(q)) = (project(corner(a)), project(corner(b))) {
                        canvas.line((p.0, p.1), (q.0, q.1), dim(OUTLINE, 0.6));
                    }
                }
            }
        }
    }
    if let Some((u, v, _)) = project(pose) {
        canvas.disk(u, v, 7.0, SENSOR);
    }
    canvas.save(path);
}

/// Ten passes of rays into a vote map, returned as published cells.
fn build_map(scans: &[Scan], speed: f64, geometry: VoxelGeometry) -> Vec<(VoxelKey, Votes)> {
    let (mut writer, reader) =
        RaySceneWriter::<_, 16>::open(geometry, &RaySceneConfig::default(), VoteModel::default())
            .unwrap();
    for scan in scans {
        let rays: Vec<RayObservation> = scan
            .offsets
            .iter()
            .zip(&scan.fired)
            .map(|(offset, fired)| RayObservation {
                source: [
                    scan.pose[0] + speed * f64::from(*fired) * 0.1,
                    scan.pose[1],
                    scan.pose[2],
                ],
                hit: [
                    scan.pose[0] + f64::from(offset[0]),
                    scan.pose[1] + f64::from(offset[1]),
                    scan.pose[2] + f64::from(offset[2]),
                ],
            })
            .collect();
        writer.insert_rays(&rays).unwrap();
        writer.publish().unwrap();
    }
    reader.snapshot_all().iter().collect()
}

/// Colour for a voxel in a slice, and its priority when two floor layers share a pixel.
type SliceColour<'a> = &'a dyn Fn(Votes) -> (Rgb, f64);

fn render_slices(
    cells: &[(VoxelKey, Votes)],
    scans: &[Scan],
    voxel: f64,
    colour: SliceColour,
    bar: bool,
    names: [&Path; 2],
) {
    let (x0, x1, y0, y1) = (-42.0, 46.0, -15.6, 15.6);
    let first_column = (x0 / voxel).floor() as i64;
    let top_row = (y1 / voxel).ceil() as i64 - 1;
    let columns = ((x1 / voxel).ceil() as i64 - first_column) as usize;
    let rows = (top_row - (y0 / voxel).floor() as i64 + 1) as usize;
    let scale = (1760 / columns).max(1);
    let margin = if bar { 70 } else { 0 };
    let one_metre_layer = (1.0 / voxel).floor() as i64;
    let mut one_metre = Canvas::new(columns * scale, rows * scale + margin, UNKNOWN);
    let mut floor = Canvas::new(columns * scale, rows * scale + margin, UNKNOWN);
    let mut floor_priority = vec![f64::NEG_INFINITY; columns * rows];
    for &(key, votes) in cells {
        let column = key.x - first_column;
        let row = top_row - key.y;
        if column < 0 || row < 0 || column as usize >= columns || row as usize >= rows {
            continue;
        }
        let (column, row) = (column as usize, row as usize);
        let (color, priority) = colour(votes);
        let fill = |canvas: &mut Canvas| {
            for dy in 0..scale {
                for dx in 0..scale {
                    canvas.set(
                        (column * scale + dx) as i64,
                        (row * scale + dy) as i64,
                        color,
                    );
                }
            }
        };
        if key.z == one_metre_layer {
            fill(&mut one_metre);
        }
        if (key.z == -1 || key.z == 0) && priority > floor_priority[row * columns + column] {
            floor_priority[row * columns + column] = priority;
            fill(&mut floor);
        }
    }
    for canvas in [&mut one_metre, &mut floor] {
        for scan in scans {
            let px = (scan.pose[0] / voxel - first_column as f64) * scale as f64;
            let py = ((top_row + 1) as f64 - scan.pose[1] / voxel) * scale as f64;
            canvas.disk(px, py, 3.0, SENSOR);
        }
        if bar {
            canvas.colour_bar();
        }
    }
    one_metre.save(names[0]);
    floor.save(names[1]);
}

/// Voxels as shaded cubes, seen from behind the sensor. `colour` returns `None` to skip a
/// voxel; it gets the votes and the height of the voxel centre.
fn render_voxels(
    cells: &[(VoxelKey, Votes)],
    geometry: &VoxelGeometry,
    world: &Warehouse,
    pose: [f64; 3],
    colour: &dyn Fn(Votes, f64) -> Option<Rgb>,
    bar: bool,
    path: &Path,
) {
    let camera = Camera::behind(pose);
    let (_, ceiling) = world.extent();
    let size = geometry.voxel_size();
    let mut canvas = Canvas::new(camera.width, camera.height, UNKNOWN);
    for &(key, votes) in cells {
        let min = geometry.voxel_lower(key);
        if min[2] >= ceiling - size {
            continue;
        }
        let max = [min[0] + size, min[1] + size, min[2] + size];
        let center = [
            min[0] + size / 2.0,
            min[1] + size / 2.0,
            min[2] + size / 2.0,
        ];
        let Some(base) = colour(votes, center[2]) else {
            continue;
        };
        let Some((_, _, depth)) = camera.project(center) else {
            continue;
        };
        for axis in 0..3 {
            let faces = [
                (min[axis], camera.eye[axis] < min[axis]),
                (max[axis], camera.eye[axis] > max[axis]),
            ];
            for (plane, visible) in faces {
                if !visible {
                    continue;
                }
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                let corner = |a: f64, b: f64| {
                    let mut point = [0.0; 3];
                    point[axis] = plane;
                    point[u] = a;
                    point[v] = b;
                    camera.project(point).map(|(x, y, _)| (x, y))
                };
                let (Some(a), Some(b), Some(c), Some(d)) = (
                    corner(min[u], min[v]),
                    corner(max[u], min[v]),
                    corner(max[u], max[v]),
                    corner(min[u], max[v]),
                ) else {
                    continue;
                };
                let shade = match axis {
                    2 if plane == max[2] => 1.0,
                    2 => 0.45,
                    0 => 0.78,
                    _ => 0.62,
                };
                canvas.fill_quad([a, b, c, d], depth, dim(base, shade));
            }
        }
    }
    if let Some((u, v, _)) = camera.project(pose) {
        canvas.disk(u, v, 7.0, SENSOR);
    }
    if bar {
        canvas.colour_bar();
    }
    canvas.save(path);
}

fn print_share_quantiles(cells: &[(VoxelKey, Votes)]) {
    let mut shares: Vec<f64> = cells
        .iter()
        .filter(|(_, votes)| votes.hits > 0)
        .map(|(_, votes)| votes.hit_share())
        .collect();
    shares.sort_unstable_by(f64::total_cmp);
    let at = |q: f64| shares[((shares.len() - 1) as f64 * q).round() as usize] * 100.0;
    println!(
        "voxels with hits: {} of {} observed; hit share p10 {:.1}% p50 {:.1}% p90 {:.1}% max {:.1}%",
        shares.len(),
        cells.len(),
        at(0.1),
        at(0.5),
        at(0.9),
        at(1.0)
    );
}

fn main() {
    let args = Args::parse();
    let rule = BlockedRule::new(args.min_hits, args.blocked_share).expect("blocked rule");
    let out = &args.out;
    std::fs::create_dir_all(out).expect("output directory");
    let world = Warehouse::new(7);
    let per_period = 30;
    let speed = PERIOD_M / per_period as f64 * 10.0;
    let band = Lidar::for_rate(2.3e6, 10.0, 40.0);
    let scans = scene::cast_period(&world, &band, per_period, [speed, 0.0, 0.0], 0.1, 7);
    render_top(&world, &scans[0], &out.join("scan-top.ppm"));
    render_perspective(&world, &scans[..3], true, &out.join("scan-perspective.ppm"));
    let mut hemisphere = band;
    hemisphere.elevation_deg = (0.0, 90.0);
    let upward = scene::cast_scan(
        &world,
        &hemisphere,
        scans[0].pose,
        [speed, 0.0, 0.0],
        0.1,
        7,
    );
    render_perspective(
        &world,
        std::slice::from_ref(&upward),
        false,
        &out.join("hemisphere-perspective.ppm"),
    );

    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], args.voxel).expect("voxel size");
    let cells = build_map(&scans[..10], speed, geometry);
    print_share_quantiles(&cells);
    let (_, ceiling) = world.extent();
    let by_height = |votes: Votes, z: f64| {
        rule.is_blocked(votes)
            .then(|| ramp(0.15 + 0.85 * (z / ceiling).clamp(0.0, 1.0)))
    };
    let by_share = |votes: Votes, _: f64| (votes.hits > 0).then(|| ramp(votes.hit_share()));
    let class = |votes: Votes| {
        if rule.is_blocked(votes) {
            (BLOCKED, 3.0)
        } else if votes.hits > 0 {
            (ERODED, 2.0)
        } else {
            (FREE, 1.0)
        }
    };
    let share = |votes: Votes| {
        if votes.hits > 0 {
            (ramp(votes.hit_share()), 1.0 + votes.hit_share())
        } else {
            (FREE, 0.0)
        }
    };
    let pose = scans[0].pose;
    render_voxels(
        &cells,
        &geometry,
        &world,
        pose,
        &by_height,
        false,
        &out.join("map-voxels.ppm"),
    );
    render_voxels(
        &cells,
        &geometry,
        &world,
        pose,
        &by_share,
        true,
        &out.join("share-voxels.ppm"),
    );
    render_slices(
        &cells,
        &scans[..10],
        args.voxel,
        &class,
        false,
        [
            &out.join("map-slice-1m.ppm"),
            &out.join("map-slice-floor.ppm"),
        ],
    );
    render_slices(
        &cells,
        &scans[..10],
        args.voxel,
        &share,
        true,
        [
            &out.join("share-slice-1m.ppm"),
            &out.join("share-slice-floor.ppm"),
        ],
    );
}
