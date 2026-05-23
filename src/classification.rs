use las::point::Classification;

use crate::{Result, VoxelizeError};

#[derive(Clone, Copy)]
pub struct ClassificationFilter {
    allowed: [bool; 256],
}

impl ClassificationFilter {
    pub fn all() -> Self {
        Self {
            allowed: [true; 256],
        }
    }

    pub fn from_codes(codes: &[u8]) -> Result<Self> {
        if codes.is_empty() {
            return Err(VoxelizeError::EmptyClassificationCodes);
        }

        let mut allowed = [false; 256];
        for &code in codes {
            if Classification::new(code).is_err() {
                return Err(VoxelizeError::InvalidClassificationCode { code });
            }
            allowed[code as usize] = true;
        }

        Ok(Self { allowed })
    }

    pub fn allows(&self, classification: Classification) -> bool {
        self.allowed[u8::from(classification) as usize]
    }
}
