#!/usr/bin/env python3
"""Offline sanity check of robot.urdf.xacro (no ROS runtime needed).

  pip install numpy trimesh python-fcl xacro
  python3 tools/verify_kinematics.py                 # from the package root

What it does
  1. expands the xacro (resolving $(find robot_arm_description) to this folder),
  2. validates the joint tree and every <inertial> (positive mass, physically valid inertia),
  3. rebuilds each STL at its visual pose from the URDF alone and reports
       - mesh-vs-mesh collisions in the q=0 home pose,
       - the contiguous collision-free angle window around q=0 for each joint
         (adjacent-link pairs only; use it to sanity-check the joint limits).
"""
import os, re, sys, tempfile, shutil
import xml.etree.ElementTree as ET
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

def expand():
    import xacro
    tmp = tempfile.mkdtemp()
    dst = os.path.join(tmp, 'robot_arm_description')
    shutil.copytree(ROOT, dst)
    for dp, _, fs in os.walk(os.path.join(dst, 'urdf')):
        for f in fs:
            p = os.path.join(dp, f); t = open(p).read()
            open(p, 'w').write(t.replace('$(find robot_arm_description)', dst))
    doc = xacro.process_file(os.path.join(dst, 'urdf', 'robot.urdf.xacro'))
    return ET.fromstring(doc.toxml()), dst

def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,          cp*cr]])

def origin(el):
    T = np.eye(4)
    o = el.find('origin') if el is not None else None
    if o is not None:
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        T[:3, :3] = rpy_mat(*rpy); T[:3, 3] = xyz
    return T

def axis_rot(ax, q):
    ax = np.array(ax, float); ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(3) + np.sin(q)*K + (1-np.cos(q))*K@K
    T = np.eye(4); T[:3, :3] = R; return T

def main():
    robot, pkg = expand()
    links = {l.get('name'): l for l in robot.findall('link')}
    joints = {j.get('name'): j for j in robot.findall('joint')}
    # ---- tree checks
    children = {j.find('child').get('link') for j in joints.values()}
    roots = [n for n in links if n not in children]
    assert roots == ['world'], f'expected single root "world", got {roots}'
    print('joint tree OK; root =', roots[0], '; joints =', list(joints))
    # ---- inertial checks
    for n, l in links.items():
        i = l.find('inertial')
        if i is None: continue
        m = float(i.find('mass').get('value')); a = i.find('inertia').attrib
        I = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]], float)
        w = np.linalg.eigvalsh(I)
        ok = m > 0 and w.min() > 0 and w[0]+w[1] >= w[2]*(1-1e-9)
        print(f'  inertial {n:15s} m={m:5.3f} kg  principal I=[{w[0]:.2e} {w[1]:.2e} {w[2]:.2e}]  valid={ok}')
        assert ok, n
    # ---- chain
    chain = [j for j in joints.values() if j.get('type') == 'revolute']
    order = []; cur = 'world'
    while True:
        nxt = [j for j in joints.values() if j.find('parent').get('link') == cur]
        if not nxt: break
        order.append(nxt[0]); cur = nxt[0].find('child').get('link')
    try:
        import trimesh
    except ImportError:
        print('trimesh not installed: skipping collision windows'); return
    meshes = {}
    for n, l in links.items():
        v = l.find('visual')
        if v is None: continue
        g = v.find('geometry/mesh'); fn = g.get('filename').replace('package://robot_arm_description', pkg)
        sc = np.array([float(s) for s in g.get('scale', '1 1 1').split()])
        m = trimesh.load(fn, force='mesh'); m.apply_scale(sc); m.apply_transform(origin(v)); meshes[n] = m
    def fk(qs):
        T = np.eye(4); out = {'world': T.copy(), 'base_link': T.copy()}
        for j, q in zip([j for j in order if j.get('type') == 'revolute'], qs):
            T = T @ origin(j) @ axis_rot([float(x) for x in j.find('axis').get('xyz').split()], q)
            out[j.find('child').get('link')] = T.copy()
        return out
    rev = [j for j in order if j.get('type') == 'revolute']
    lim = [(float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))) for j in rev]
    cm = trimesh.collision.CollisionManager()
    for n, m in meshes.items(): cm.add_object(n, m)
    def hits(q):
        F = fk(q)
        for n in meshes: cm.set_transform(n, F[n])
        return {tuple(sorted(p)) for p in cm.in_collision_internal(return_names=True)[1]}
    # disc rests flush on the rim -> ignore that coplanar contact pair
    ignore = {('base_link', 'shoulder_link')}
    print('home-pose collisions (excluding flush base/shoulder seat):', hits([0]*len(rev)) - ignore or 'none')
    adj = [{tuple(sorted((j.find('parent').get('link'), j.find('child').get('link'))))} for j in rev]
    for k, j in enumerate(rev):
        def free(deg):
            q = [0.0]*len(rev); q[k] = np.radians(deg); return not ((hits(q) - ignore) & adj[k])
        hi = 0.0
        while hi < 180 and free(hi+1): hi += 1
        lo = 0.0
        while lo > -180 and free(lo-1): lo -= 1
        l, u = np.degrees(lim[k])
        status = 'OK' if lo <= l and hi >= u else 'LIMIT EXCEEDS FREE WINDOW'
        if hi >= 180 and lo <= -180: status = 'no geometric stop (limit is a servo assumption)'
        print(f'  {j.get("name"):20s} free window [{lo:6.0f}, {hi:5.0f}] deg | limits [{l:6.1f}, {u:6.1f}] -> {status}')

if __name__ == '__main__':
    main()
