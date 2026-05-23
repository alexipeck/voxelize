use anyhow::{bail, Result};
use las::point::Classification;

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
            bail!("at least one classification code is required");
        }

        let mut allowed = [false; 256];
        for &code in codes {
            Classification::new(code).map_err(|_| {
                anyhow::anyhow!("invalid classification code: {code}")
            })?;
            allowed[code as usize] = true;
        }

        Ok(Self { allowed })
    }

    pub fn allows(&self, classification: Classification) -> bool {
        self.allowed[u8::from(classification) as usize]
    }
}
