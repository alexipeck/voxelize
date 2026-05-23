use std::io::Write;
use std::path::PathBuf;
use clap::Parser;
use voxelaz::{CancellationToken, ClassificationFilter, VoxelizeConfig, VoxelizeError};

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
    let cancel = CancellationToken::new();
    let cancel_for_handler = cancel.clone();
    ctrlc::set_handler(move || cancel_for_handler.cancel())
        .expect("failed to install Ctrl-C handler");
    let mut config = VoxelizeConfig::new(cli.voxel_size);
    config.classification_filter = classification_filter;
    config.counts_as_intensity = cli.counts_as_intensity;
    config.cancel = Some(cancel);
    match voxelaz::voxelize(&cli.input, &cli.output, &config) {
        Ok(()) => Ok(()),
        Err(err) => {
            if matches!(err, VoxelizeError::Cancelled) {
                let _ = writeln!(std::io::stderr(), "cancelled");
                std::process::exit(130);
            }
            Err(err.into())
        }
    }
}
