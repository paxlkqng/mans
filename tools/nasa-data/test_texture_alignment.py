"""Independent local HiRISE affine and bilinear sampling checks."""
import unittest
from pathlib import Path
import numpy as np
import rasterio
from rasterio.windows import Window

LOCAL = Path(__file__).resolve().parents[2] / "data/local"


@unittest.skipUnless((LOCAL / "hirise-jezero/ortho.tif").exists(), "local HiRISE assets absent")
class AlignmentTests(unittest.TestCase):
    def test_georeferenced_image_samples(self):
        with rasterio.open(LOCAL / "hirise-jezero/elevation.tif") as dtm, rasterio.open(LOCAL / "hirise-jezero/ortho.tif") as image, rasterio.open(LOCAL / "ESP_045994_1985_RED_C_01_ORTHO.JP2") as source:
            self.assertEqual(dtm.transform, image.transform)
            self.assertEqual(dtm.crs, image.crs)
            self.assertEqual(dtm.shape, image.shape)
            output = image.read(1)
            for row in [8, 100, 256, 400, 500]:
                for col in [8, 100, 256, 400, 500]:
                    x, y = dtm.xy(row, col)
                    px, py = ~source.transform * (x, y)
                    px, py = px - 0.5, py - 0.5
                    left, top = int(np.floor(px)), int(np.floor(py))
                    patch = source.read(1, window=Window(left, top, 2, 2)).astype(float)
                    if np.any(patch == 0):
                        continue
                    fx, fy = px-left, py-top
                    expected = (patch[0, 0]*(1-fx)*(1-fy) + patch[0, 1]*fx*(1-fy)
                                + patch[1, 0]*(1-fx)*fy + patch[1, 1]*fx*fy)
                    self.assertLessEqual(abs(int(output[row, col])-expected), 1.0)

    def test_elevation_unchanged(self):
        with rasterio.open(LOCAL / "hirise-jezero/elevation.tif") as dtm, rasterio.open(LOCAL / "DTEEC_045994_1985_046060_1985_U01.IMG") as source:
            np.testing.assert_array_equal(dtm.read(1), source.read(1, window=Window(3000, 4000, 512, 512)))


if __name__ == "__main__":
    unittest.main()
