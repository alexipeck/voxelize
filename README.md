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
