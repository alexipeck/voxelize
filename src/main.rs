use std::path::PathBuf;

use clap::Parser;

#[derive(Parser)]
struct Cli {
    input: PathBuf,
    output: PathBuf,
    #[arg(long)]
    voxel_size: f64,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    voxelize::voxelize(&cli.input, &cli.output, cli.voxel_size)
}
