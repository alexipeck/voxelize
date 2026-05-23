# voxelize

Voxelize LAS/LAZ point clouds into a downsampled output where each point is a voxel center.

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
use voxelize::{ClassificationFilter, VoxelizeConfig};

let config = VoxelizeConfig {
    voxel_size: 0.5,
    classification_filter: ClassificationFilter::all(),
    counts_as_intensity: false,
};
voxelize::voxelize(input, output, &config)?;
```

`VoxelizeConfig::new(voxel_size)` sets `classification_filter` to all codes and `counts_as_intensity` to `false`.
