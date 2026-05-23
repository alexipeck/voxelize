use thiserror::Error;

pub type Result<T> = std::result::Result<T, VoxelizeError>;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum VoxelizeError {
    #[error("voxel_size must be greater than 0")]
    InvalidVoxelSize,

    #[error("at least one classification code is required")]
    EmptyClassificationCodes,

    #[error("invalid classification code: {code}")]
    InvalidClassificationCode { code: u8 },

    #[error(transparent)]
    Las(#[from] las::Error),

    #[error("consumer dropped before producer finished")]
    ConsumerDropped,

    #[error("buffer pool send failed")]
    BufferPoolSendFailed,

    #[error("consumer thread panicked")]
    ConsumerPanicked,

    #[error("producer thread panicked")]
    ProducerPanicked,

    #[error("operation cancelled")]
    Cancelled,
}

pub(crate) fn join_consumer(handle: std::thread::JoinHandle<Result<()>>) -> Result<()> {
    match handle.join() {
        Ok(result) => result,
        Err(_) => Err(VoxelizeError::ConsumerPanicked),
    }
}

pub(crate) fn join_producer(handle: std::thread::JoinHandle<Result<()>>) -> Result<()> {
    match handle.join() {
        Ok(result) => result,
        Err(_) => Err(VoxelizeError::ProducerPanicked),
    }
}
