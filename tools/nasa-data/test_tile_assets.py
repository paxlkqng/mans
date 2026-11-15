"""Check observed 3x3 tile continuity, masks, and shared-halo normals."""
import json
from pathlib import Path
import unittest
import numpy as np
import rasterio

ROOT=Path(__file__).resolve().parents[2]/"data/local/hirise-tiles"

@unittest.skipUnless((ROOT/"manifest.json").exists(),"local tile assets absent")
class TileAssets(unittest.TestCase):
    def test_shared_edges_and_coordinates(self):
        m=json.loads((ROOT/"manifest.json").read_text())
        tiles={}
        for t in m["tiles"]:
            p=ROOT/t["path"]
            with rasterio.open(p/"elevation.tif") as d, rasterio.open(p/"ortho.tif") as o:
                self.assertEqual(d.transform,o.transform)
                self.assertEqual(d.crs,o.crs)
                self.assertEqual(d.shape,(512,512))
                mask=np.fromfile(p/"valid.u8",dtype="u1").reshape(512,512)
                self.assertEqual(int(mask.sum()),t["validSamples"])
                tiles[t["x"],t["z"]]=(t,d.read(1),o.read(1),np.fromfile(p/"normals.f32",dtype="<f4").reshape(512,512,3))
        self.assertEqual(len(tiles),9)
        for (x,z),(t,h,o,n) in tiles.items():
            if (x+1,z) in tiles:
                b,hh,oo,nn=tiles[x+1,z]
                self.assertAlmostEqual(t["bounds"][2],b["bounds"][0],places=7)
                np.testing.assert_array_equal(h[:,-1],hh[:,0])
                np.testing.assert_array_equal(o[:,-1],oo[:,0])
                np.testing.assert_array_equal(n[:,-1],nn[:,0])
            if (x,z+1) in tiles:
                b,hh,oo,nn=tiles[x,z+1]
                self.assertAlmostEqual(t["bounds"][1],b["bounds"][3],places=7)
                np.testing.assert_array_equal(h[-1],hh[0])
                np.testing.assert_array_equal(o[-1],oo[0])
                np.testing.assert_array_equal(n[-1],nn[0])

if __name__=="__main__":unittest.main()
