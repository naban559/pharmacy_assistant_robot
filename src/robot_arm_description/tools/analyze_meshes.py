#!/usr/bin/env python3
"""Reproduce the STL-derived numbers used in robot.urdf.xacro.

  pip install numpy scipy pillow
  python3 tools/analyze_meshes.py

Prints, per mesh (millimetres, mesh frame): bounding box, solid volume, volume centroid, and every
enclosed through-hole seen along X, Y and Z (centre, diameter).  These holes are the servo shafts,
hinge pins and bolt holes that define the joint axes.
"""
import os, re, struct, sys
import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

MESH_DIR = os.path.join(os.path.dirname(__file__), '..', 'meshes')

def load(path):
    b = open(path, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:1000]:
        return np.array(re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b.decode(errors='ignore')), float).reshape(-1, 3, 3)
    n = struct.unpack('<I', b[80:84])[0]
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(b, dtype=dt, count=n, offset=84)['v'].astype(float)

def volume_centroid(t):
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    return v.sum(), ((a + b + c) * (v[:, None] / 4)).sum(0) / v.sum()

def holes(t, h, v, res=0.25, min_r=0.8):
    pts = t[:, :, [h, v]]
    mn = pts.reshape(-1, 2).min(0) - 2; mx = pts.reshape(-1, 2).max(0) + 2
    W = int((mx[0]-mn[0])/res)+1; H = int((mx[1]-mn[1])/res)+1
    im = Image.new('L', (W, H), 0); d = ImageDraw.Draw(im)
    for tri in pts: d.polygon([((x-mn[0])/res, (mx[1]-y)/res) for x, y in tri], fill=255)
    lab, n = ndimage.label(~(np.array(im) > 0)); out = []
    for i in range(1, n+1):
        ys, xs = np.where(lab == i)
        if xs.min() == 0 or ys.min() == 0 or xs.max() == W-1 or ys.max() == H-1: continue
        r = np.sqrt(len(xs)*res*res/np.pi)
        if r >= min_r: out.append((round(mn[0]+xs.mean()*res, 1), round(mx[1]-ys.mean()*res, 1), round(2*r, 1)))
    return out

for name in ['base', 'shoulder', 'upper_arm', 'fore_arm', 'wrist']:
    t = load(os.path.join(MESH_DIR, name + '.stl')); p = t.reshape(-1, 3)
    V, c = volume_centroid(t)
    print(f'\n{name}: bbox min {p.min(0).round(1)} max {p.max(0).round(1)}  volume {V:,.0f} mm^3  centroid {c.round(1)}')
    for (h, v), lab in [((0, 1), 'along Z (x,y)'), ((0, 2), 'along Y (x,z)'), ((1, 2), 'along X (y,z)')]:
        hs = holes(t, h, v)
        if hs: print(f'   through-holes {lab}: ' + '; '.join(f'centre ({a},{b}) dia {d}' for a, b, d in hs))
