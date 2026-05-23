use std::path::PathBuf;

use clap::Parser;
use voxelize::ClassificationFilter;

#[derive(Parser)]
struct Cli {
    input: PathBuf,
    output: PathBuf,
    #[arg(long)]
    voxel_size: f64,
    #[arg(long, value_delimiter = ',')]
    include_classification: Option<Vec<u8>>,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let classification_filter = match cli.include_classification {
        None => ClassificationFilter::all(),
        Some(codes) => ClassificationFilter::from_codes(&codes)?,
    };
    voxelize::voxelize(
        &cli.input,
        &cli.output,
        cli.voxel_size,
        classification_filter,
    )
}
