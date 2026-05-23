use std::path::PathBuf;

use clap::Parser;
use voxelaz::{ClassificationFilter, VoxelizeConfig};

#[derive(Parser)]
struct Cli {
    #[arg(long)]
    input: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long)]
    voxel_size: f64,
    #[arg(long, value_delimiter = ',')]
    include_classification: Option<Vec<u8>>,
    #[arg(long, default_value_t = false)]
    counts_as_intensity: bool,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let classification_filter = match cli.include_classification {
        None => ClassificationFilter::all(),
        Some(codes) => ClassificationFilter::from_codes(&codes)?,
    };
    let mut config = VoxelizeConfig::new(cli.voxel_size);
    config.classification_filter = classification_filter;
    config.counts_as_intensity = cli.counts_as_intensity;
    voxelaz::voxelize(&cli.input, &cli.output, &config)?;
    Ok(())
}
