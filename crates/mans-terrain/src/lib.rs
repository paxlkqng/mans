use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TerrainAsset {
    pub size: usize,
    pub cell_size: f64,
    pub heights: Vec<f64>,
    pub source: String,
    pub provenance: String,
}

#[derive(Debug)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl TerrainAsset {
    pub fn validate(&self) -> Result<(), String> {
        if self.size < 2 {
            return Err("terrain grid must be at least 2×2".into());
        }
        if !self.cell_size.is_finite() || self.cell_size <= 0.0 {
            return Err("cellSize must be a positive finite value in meters".into());
        }
        if self.heights.len() != self.size * self.size {
            return Err("height count does not match grid dimensions".into());
        }
        if self.heights.iter().any(|h| !h.is_finite()) {
            return Err("terrain contains invalid elevation samples".into());
        }
        Ok(())
    }
    pub fn sample(&self, x: usize, z: usize) -> Option<f64> {
        self.heights.get(z.checked_mul(self.size)? + x).copied()
    }
    pub fn mesh(&self, exaggeration: f32) -> Result<Mesh, String> {
        self.validate()?;
        if !exaggeration.is_finite() || exaggeration <= 0.0 {
            return Err("vertical exaggeration must be positive".into());
        }
        let mut positions = Vec::with_capacity(self.size * self.size);
        for z in 0..self.size {
            for x in 0..self.size {
                positions.push([
                    x as f32 * self.cell_size as f32,
                    self.sample(x, z).unwrap() as f32 * exaggeration,
                    z as f32 * self.cell_size as f32,
                ]);
            }
        }
        let mut indices = Vec::with_capacity((self.size - 1) * (self.size - 1) * 6);
        for z in 0..self.size - 1 {
            for x in 0..self.size - 1 {
                let a = (z * self.size + x) as u32;
                indices.extend_from_slice(&[
                    a,
                    a + 1,
                    a + self.size as u32,
                    a + 1,
                    a + self.size as u32 + 1,
                    a + self.size as u32,
                ]);
            }
        }
        Ok(Mesh { positions, indices })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_and_builds_mesh() {
        let t = TerrainAsset {
            size: 2,
            cell_size: 1.,
            heights: vec![0.; 4],
            source: "test".into(),
            provenance: "synthetic test only".into(),
        };
        let m = t.mesh(1.).unwrap();
        assert_eq!(m.positions.len(), 4);
        assert_eq!(m.indices.len(), 6);
    }
    #[test]
    fn rejects_bad_grid() {
        let t = TerrainAsset {
            size: 2,
            cell_size: 1.,
            heights: vec![0.; 3],
            source: "x".into(),
            provenance: "x".into(),
        };
        assert!(t.validate().is_err());
    }
}
