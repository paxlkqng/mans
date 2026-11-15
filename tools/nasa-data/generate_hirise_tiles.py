#!/usr/bin/env python3
"""Tile measured HiRISE data: 512 sample grids share one boundary sample.

511-cell stride avoids duplicated surface area and missing boundary cells.
Normals are computed from a shared source halo, so tile lighting is continuous.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
import rasterio
from rasterio.windows import Window
from rasterio.warp import reproject, Resampling, transform as project_coordinates


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--dtm",default="data/local/DTEEC_045994_1985_046060_1985_U01.IMG")
    p.add_argument("--ortho",default="data/local/ESP_045994_1985_RED_C_01_ORTHO.JP2")
    p.add_argument("--output-dir",default="data/local/hirise-tiles")
    p.add_argument("--row",type=int,default=3489)
    p.add_argument("--col",type=int,default=2489)
    p.add_argument("--count",type=int,default=3)
    a=p.parse_args()
    output=Path(a.output_dir);output.mkdir(parents=True,exist_ok=True)
    size=512;stride=size-1;total=a.count*stride+1
    with rasterio.open(a.dtm) as d:
        h=d.read(1,window=Window(a.col-1,a.row-1,total+2,total+2),masked=True,boundless=True).filled(np.nan).astype(np.float64)
        core=h[1:-1,1:-1]
        finite=np.isfinite(core)&(core>-1e10)
        if not finite.any():raise ValueError("No valid elevations in region")
        lo,hi=float(core[finite].min()),float(core[finite].max())
        cell=float(d.res[0]);transform=d.window_transform(Window(a.col,a.row,total,total))
        dx=(h[1:-1,2:]-h[1:-1,:-2])/(2*cell)
        dz=(h[2:,1:-1]-h[:-2,1:-1])/(2*cell)
        normals=np.stack([-dx,np.ones_like(dx),-dz],axis=-1)
        normals/=np.linalg.norm(normals,axis=-1,keepdims=True)
        normals[~np.isfinite(normals).all(axis=-1)]=[0,1,0]
        # One regional warp avoids per-window floating rounding differences at
        # shared edges. This is bounded to the requested region, not the source.
        with rasterio.open(a.ortho) as image:
            source_window=Window(a.col-4,a.row-4,total+8,total+8)
            pixels=image.read(1,window=source_window,boundless=True,fill_value=0)
            regional_image=np.zeros((total,total),dtype=np.uint8)
            reproject(source=pixels,destination=regional_image,
                      src_transform=image.window_transform(source_window),src_crs=d.crs,
                      dst_transform=transform,dst_crs=d.crs,src_nodata=0,dst_nodata=0,resampling=Resampling.bilinear)
        entries=[]
        for z in range(a.count):
            for x in range(a.count):
                name=f"tile-{x}-{z}";folder=output/name
                subprocess.run([sys.executable,str(Path(__file__).with_name("extract_hirise_subset.py")),
                    "--dtm",a.dtm,"--ortho",a.ortho,"--row",str(a.row+z*stride),"--col",str(a.col+x*stride),
                    "--size",str(size),"--output-dir",str(folder),"--allow-nodata"],check=True)
                sub=core[z*stride:z*stride+size,x*stride:x*stride+size]
                valid=np.isfinite(sub)&(sub>-1e10)
                with rasterio.open(folder/"ortho.tif","r+") as image:
                    image.write(regional_image[z*stride:z*stride+size,x*stride:x*stride+size],1)
                provenance=json.loads((folder/"provenance.json").read_text())
                provenance["processing"]+=" Tiled output: single bounded regional bilinear warp sliced into shared-edge tiles, ensuring identical boundary DN values."
                (folder/"provenance.json").write_text(json.dumps(provenance,indent=2)+"\n")
                normals[z*stride:z*stride+size,x*stride:x*stride+size].astype("<f4").tofile(folder/"normals.f32")
                left,top=transform*(x*stride+0.5,z*stride+0.5)
                right,bottom=transform*(x*stride+size-0.5,z*stride+size-0.5)
                lon,lat=project_coordinates(d.crs,"+proj=longlat +a=3394839.8133163 +b=3394839.8133163 +no_defs",[left,right],[bottom,top])
                entries.append(dict(id=name,x=x,z=z,path=name,bounds=[left,bottom,right,top],
                                    geographicBounds=[min(lon),min(lat),max(lon),max(lat)],
                                    validSamples=int(valid.sum()),status="valid" if valid.all() else "partial" if valid.any() else "missing"))
        ox,oy=transform*(total/2,total/2)
        manifest=dict(schemaVersion=1,name="HiRISE Jezero 3x3 measured terrain",size=size,stride=stride,count=a.count,
                      cellSize=cell,minElevation=lo,maxElevation=hi,origin=[ox,oy,(lo+hi)/2],
                      crs=str(d.crs),transform=list(transform),tiles=entries,
                      geographicCoordinates="Planetocentric latitude; east-positive longitude, degrees",
                      processing="512 samples per tile, shared 1-sample edges (511 cells); elevations unchanged, common-halo normals; ortho bilinear on DTM grid. See per-tile provenance.json.",
                      sourceDtm=Path(a.dtm).name,sourceOrtho=Path(a.ortho).name)
        (output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        print(f"Generated {len(entries)} georeferenced tiles; {total}x{total} unique samples.")

if __name__=="__main__":main()
