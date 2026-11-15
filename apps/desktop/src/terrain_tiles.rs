use crate::terrain_renderer::{load_orthophoto, Orthophoto};
use serde::Deserialize;
use std::{
    collections::{HashMap, HashSet},
    path::{Path, PathBuf},
    sync::mpsc,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TileEntry {
    pub id: String,
    pub x: usize,
    pub z: usize,
    pub path: String,
    pub bounds: [f64; 4],
    pub valid_samples: usize,
    pub status: String,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Manifest {
    pub name: String,
    pub size: usize,
    pub stride: usize,
    pub count: usize,
    pub cell_size: f64,
    pub min_elevation: f64,
    pub max_elevation: f64,
    pub origin: [f64; 3],
    pub crs: String,
    pub tiles: Vec<TileEntry>,
}
impl Manifest {
    pub fn read(path: &Path) -> Result<Self, String> {
        let m: Self =
            serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if m.size < 2
            || m.stride != m.size - 1
            || m.count == 0
            || !m.cell_size.is_finite()
            || m.cell_size <= 0.0
            || m.tiles.len() != m.count * m.count
        {
            return Err("Invalid tile manifest grid".into());
        }
        if m.crs.is_empty()
            || m.tiles.iter().any(|t| {
                t.bounds.iter().any(|v| !v.is_finite()) || t.x >= m.count || t.z >= m.count
            })
        {
            return Err("Invalid planetary CRS or tile bounds".into());
        }
        let span = m.stride as f64 * m.cell_size;
        for t in &m.tiles {
            let left = m.origin[0] - m.extent() as f64 / 2.0 + t.x as f64 * span;
            let top = m.origin[1] + m.extent() as f64 / 2.0 - t.z as f64 * span;
            // Extent is a GPU f32; allow its sub-millimeter rounding here.
            if (t.bounds[0] - left).abs() > 0.001
                || (t.bounds[3] - top).abs() > 0.001
                || ((t.bounds[2] - t.bounds[0]) - span).abs() > 0.001
                || ((t.bounds[3] - t.bounds[1]) - span).abs() > 0.001
            {
                return Err(format!(
                    "{}: projected tile bounds disagree with grid",
                    t.id
                ));
            }
        }
        Ok(m)
    }
    pub fn extent(&self) -> f32 {
        (self.count * self.stride) as f32 * self.cell_size as f32
    }
    pub fn wanted(&self, focus: [f32; 2], distance: f32, capacity: usize) -> Vec<usize> {
        let span = self.stride as f32 * self.cell_size as f32;
        let cx = ((focus[0] + self.extent() / 2.0) / span)
            .floor()
            .clamp(0.0, (self.count - 1) as f32) as isize;
        let cz = ((focus[1] + self.extent() / 2.0) / span)
            .floor()
            .clamp(0.0, (self.count - 1) as f32) as isize;
        let overview = distance > span * 2.0;
        let mut ids: Vec<usize> = self
            .tiles
            .iter()
            .enumerate()
            .filter(|(_, t)| {
                overview || ((t.x as isize - cx).abs() <= 1 && (t.z as isize - cz).abs() <= 1)
            })
            .map(|(i, _)| i)
            .collect();
        ids.sort_by_key(|i| {
            let t = &self.tiles[*i];
            (t.x as isize - cx).pow(2) + (t.z as isize - cz).pow(2)
        });
        ids.truncate(capacity.max(1));
        ids
    }
}

pub struct LoadedTile {
    pub picking: crate::terrain_picking::PickMesh,
    pub index: usize,
    pub heights: Vec<f64>,
    pub normals: Vec<[f32; 3]>,
    pub valid: Vec<u8>,
    pub image: Option<Orthophoto>,
    pub image_warning: Option<String>,
    pub prepared: Option<crate::terrain_renderer::Mesh>,
    pub coarse: Vec<u32>,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Grid {
    size: usize,
    cell_size: f64,
    heights: Vec<f64>,
}
fn load(root: &Path, m: &Manifest, index: usize) -> Result<LoadedTile, String> {
    let entry = &m.tiles[index];
    let folder = root.join(&entry.path);
    let g: Grid = serde_json::from_str(
        &std::fs::read_to_string(folder.join("terrain.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    if g.size != m.size
        || g.heights.len() != m.size * m.size
        || (g.cell_size - m.cell_size).abs() > 1e-8
    {
        return Err(format!("{}: grid differs from manifest", entry.id));
    }
    let valid = std::fs::read(folder.join("valid.u8")).map_err(|e| e.to_string())?;
    let bytes = std::fs::read(folder.join("normals.f32")).map_err(|e| e.to_string())?;
    if valid.len() != g.heights.len() || bytes.len() != g.heights.len() * 12 {
        return Err("Invalid mask/normal asset length".into());
    }
    let normals: Vec<[f32; 3]> = bytes
        .chunks_exact(12)
        .map(|b| {
            [
                f32::from_le_bytes(b[0..4].try_into().unwrap()),
                f32::from_le_bytes(b[4..8].try_into().unwrap()),
                f32::from_le_bytes(b[8..12].try_into().unwrap()),
            ]
        })
        .collect();
    let image = load_orthophoto(&folder, g.size);
    let image_warning = image.as_ref().err().cloned();
    let mut samples = g.heights.clone();
    for (h, v) in samples.iter_mut().zip(&valid) {
        if *v == 0 {
            *h = f64::NAN;
        }
    }
    let mut prepared = crate::terrain_renderer::mesh(m.size, m.cell_size, &samples)?;
    let min = samples
        .iter()
        .copied()
        .filter(|h| h.is_finite())
        .fold(f64::INFINITY, f64::min);
    let max = samples
        .iter()
        .copied()
        .filter(|h| h.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);
    let span = (m.stride as f64 * m.cell_size) as f32;
    for (i, v) in prepared.vertices.iter_mut().enumerate() {
        v.position[0] += entry.x as f32 * span - m.extent() / 2.0 + span / 2.0;
        v.position[2] += entry.z as f32 * span - m.extent() / 2.0 + span / 2.0;
        v.position[1] += ((min + max) / 2.0 - m.origin[2]) as f32;
        v.height = ((g.heights[i] - m.min_elevation) / (m.max_elevation - m.min_elevation).max(1.0))
            as f32;
        v.normal = normals[i];
    }
    let coarse = crate::terrain_renderer::coarse_indices(m.size, 4, &valid);
    let picking = crate::terrain_picking::PickMesh::new(&prepared, &coarse, m.size);
    Ok(LoadedTile {
        picking,
        index,
        heights: g.heights,
        normals,
        valid,
        image: image.ok(),
        image_warning,
        prepared: Some(prepared),
        coarse,
    })
}

pub struct TileManager {
    pub manifest: Manifest,
    requests: mpsc::SyncSender<usize>,
    results: std::sync::Mutex<mpsc::Receiver<(usize, Result<LoadedTile, String>)>>,
    pub pending: HashSet<usize>,
    pub errors: HashMap<usize, String>,
}
impl TileManager {
    pub fn new(path: &Path) -> Result<Self, String> {
        let manifest = Manifest::read(path)?;
        let root: PathBuf = path.parent().ok_or("Manifest has no directory")?.into();
        let (requests, rx) = mpsc::sync_channel(1);
        let (tx, results) = mpsc::sync_channel(2);
        let m = manifest.clone();
        std::thread::Builder::new()
            .name("mans-terrain-loader".into())
            .spawn(move || {
                while let Ok(i) = rx.recv() {
                    let result = load(&root, &m, i);
                    if tx.send((i, result)).is_err() {
                        break;
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            manifest,
            requests,
            results: std::sync::Mutex::new(results),
            pending: HashSet::new(),
            errors: HashMap::new(),
        })
    }
    pub fn request(&mut self, ids: &[usize], loaded: &HashSet<usize>) {
        for i in ids {
            if self.manifest.tiles[*i].valid_samples == 0
                || self.manifest.tiles[*i].status == "missing"
            {
                self.errors
                    .entry(*i)
                    .or_insert("Tile has no observed elevation samples".into());
                continue;
            }
            if loaded.contains(i) || self.pending.contains(i) || self.errors.contains_key(i) {
                continue;
            }
            if self.requests.try_send(*i).is_ok() {
                self.pending.insert(*i);
            } else {
                break;
            }
        }
    }
    pub fn poll(&mut self) -> Option<LoadedTile> {
        while let Ok((i, result)) = self.results.get_mut().unwrap().try_recv() {
            self.pending.remove(&i);
            match result {
                Ok(tile) => return Some(tile),
                Err(e) => {
                    tracing::warn!(tile=i,error=%e,"tile unavailable");
                    self.errors.insert(i, e);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn camera_selection_is_bounded_and_includes_current_tile() {
        let mut m = Manifest {
            name: "test".into(),
            size: 512,
            stride: 511,
            count: 3,
            cell_size: 1.0,
            min_elevation: 0.0,
            max_elevation: 10.0,
            origin: [0.0; 3],
            crs: "test metric".into(),
            tiles: vec![],
        };
        for z in 0..3 {
            for x in 0..3 {
                m.tiles.push(TileEntry {
                    id: format!("{x}-{z}"),
                    x,
                    z,
                    path: String::new(),
                    bounds: [0.0; 4],
                    valid_samples: 1,
                    status: "valid".into(),
                });
            }
        }
        assert_eq!(m.wanted([0.0, 0.0], 3000.0, 9).len(), 9);
        let wanted = m.wanted([-600.0, -600.0], 200.0, 9);
        assert_eq!(wanted.len(), 4);
        assert_eq!(wanted[0], 0);
        assert_eq!(m.wanted([0.0, 0.0], 3000.0, 4).len(), 4);
    }
}
