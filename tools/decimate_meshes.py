#!/usr/bin/env python3
"""Decimate the meshes a robot URDF references into Unity-friendly binary STLs.

Setup (host python is externally managed, use a venv):
    python3 -m venv /tmp/venv && /tmp/venv/bin/pip install numpy trimesh fast-simplification pycollada
Run:
    /tmp/venv/bin/python tools/decimate_meshes.py --robot sobit_home     (default; or sobit_light)

Inputs : tools/models/<robot>/{<robot>.urdf, budget.json}
         tools/models/<robot>/src/meshes/**  (copy of <robot>_description/meshes, referenced files only)
         tools/models/<robot>/src/ext/<pkg>/<file>  (meshes of other packages, optional; e.g.
         docker cp <container>:/home/<user>/colcon_ws/src/realsense_ros/realsense2_description/meshes/d415.stl
         tools/models/sobit_home/src/ext/realsense2_description/  -- 21 MB, not kept in git)
         URIs: package://<pkg>/meshes/<rel> or file:///.../share/<pkg>/meshes/<rel>; own package = <robot>_description
Outputs: tools/models/<robot>/meshes_lod/<rel path>[.<material>].stl and colors.json
Units/axes are those of the source (ROS axes; STL in the units stored, DAE converted to metres through
<unit meter> and node transforms). The Unity builder applies the URDF <scale> and the ROS->Unity axis change.
"""
import argparse, json, os, re, struct, sys, collections
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import fast_simplification

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT = "sobit_home"
MODEL = SRC = URDF = OUT = BUDGET = None


def configure(robot):
    global ROBOT, MODEL, SRC, URDF, OUT, BUDGET
    ROBOT = robot
    MODEL = os.path.join(ROOT, "tools", "models", robot)
    SRC = os.path.join(MODEL, "src")
    URDF = os.path.join(MODEL, robot + ".urdf")
    OUT = os.path.join(MODEL, "meshes_lod")
    BUDGET = json.load(open(os.path.join(MODEL, "budget.json")))


def src_path(rel):
    p = os.path.join(SRC, "meshes", rel)
    if rel.startswith("ext/"):
        p = os.path.join(SRC, rel)
    return p


def referenced():
    """rel path (relative to meshes/, or ext/<pkg>/<file>) -> number of URDF instances"""
    cnt = collections.Counter()
    for m in ET.parse(URDF).getroot().iter("mesh"):
        f = m.get("filename")
        mm = re.match(r"package://([^/]+)/meshes/(.*)", f) or re.match(r"file://.*/share/([^/]+)/meshes/(.*)", f)
        if not mm:
            continue
        if mm.group(1) == ROBOT + "_description":
            cnt[mm.group(2)] += 1
        else:
            cnt["ext/%s/%s" % (mm.group(1), mm.group(2))] += 1
    return cnt


def write_stl(path, verts, faces):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    v = np.asarray(verts, dtype=np.float32)
    f = np.asarray(faces, dtype=np.int64)
    tri = v[f]  # n,3,3
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 0, n / np.maximum(ln, 1e-30), 0).astype(np.float32)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = tri
    with open(path, "wb") as fh:
        fh.write(b"decimated by decimate_meshes.py".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(f)))
        fh.write(rec.tobytes())


def cluster(v, f, cell):
    """Grid vertex clustering (vertices of one cell collapse to their mean)."""
    v = np.asarray(v, dtype=np.float64)
    q = np.floor((v - v.min(0)) / cell).astype(np.int64)
    _, inv, cnt = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    nv = np.zeros((len(cnt), 3))
    np.add.at(nv, inv, v)
    nv /= cnt[:, None]
    nf = inv[f]
    ok = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
    nf = nf[ok]
    key = np.sort(nf, axis=1)
    _, first = np.unique(key, axis=0, return_index=True)
    nf = nf[np.sort(first)]
    return nv.astype(np.float32), nf.astype(np.int32)


def decimate(verts, faces, target):
    v = np.asarray(verts, dtype=np.float32)
    f = np.asarray(faces, dtype=np.int32)
    if len(f) <= target:
        return v, f
    # fast_simplification preserves open boundaries / non-manifold seams and can stall above the target
    for agg in (7, 9, 10):
        for _ in range(3):
            if len(f) <= target * 1.05:
                return v, f
            before = len(f)
            v, f = fast_simplification.simplify(v, f, target_count=int(target), agg=agg)
            if len(f) > before * 0.98:
                break
    if len(f) <= target * 1.05:
        return v, f
    # still above target: grid vertex clustering, bisect the cell size
    diag = float(np.linalg.norm(v.max(0) - v.min(0)))
    lo, hi = diag * 1e-5, diag * 0.2
    best = None
    for _ in range(24):
        mid = (lo * hi) ** 0.5
        cv, cf = cluster(v, f, mid)
        if len(cf) > target:
            lo = mid
        else:
            hi = mid
            best = (cv, cf)
    if best is None:
        best = cluster(v, f, hi)
    print("    (clustering fallback: %d -> %d)" % (len(f), len(best[1])))
    return best


def load_stl(path):
    m = trimesh.load(path, file_type="stl", process=True)  # merges identical vertices
    m.remove_unreferenced_vertices()
    return np.asarray(m.vertices), np.asarray(m.faces)


def load_dae(path):
    import collada
    col = collada.Collada(path)
    unit = col.assetInfo.unitmeter or 1.0
    parts = collections.OrderedDict()  # material name -> (verts list, faces list, rgba)
    for geom in col.scene.objects("geometry"):  # node transforms already applied
        for prim in geom.primitives():
            if hasattr(prim, "triangleset"):
                prim = prim.triangleset()
            mat = prim.material
            name = (mat.name if mat is not None and getattr(mat, "name", None) else "default") if mat else "default"
            name = re.sub(r"[^A-Za-z0-9_\-]", "_", name)
            rgba = [0.6, 0.6, 0.6, 1.0]
            if mat is not None and getattr(mat, "effect", None) is not None:
                d = getattr(mat.effect, "diffuse", None)
                if isinstance(d, (tuple, list)) and len(d) >= 3:
                    rgba = [float(x) for x in d] + ([1.0] if len(d) == 3 else [])
            idx = prim.vertex_index  # n,3
            verts = np.asarray(prim.vertex, dtype=np.float64) * unit
            pv, pf, _ = parts.setdefault(name, ([], [], rgba))
            off = sum(len(x) for x in pv)
            pv.append(verts)
            pf.append(idx + off)
    out = {}
    for name, (pv, pf, rgba) in parts.items():
        v = np.concatenate(pv)
        f = np.concatenate(pf)
        m = trimesh.Trimesh(v, f, process=True)
        m.remove_unreferenced_vertices()
        out[name] = (np.asarray(m.vertices), np.asarray(m.faces), rgba)
    return out


def fmt_b(v):
    return "[%.3f %.3f %.3f]..[%.3f %.3f %.3f]" % (*v.min(0), *v.max(0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="sobit_home")
    configure(ap.parse_args().robot)
    inst = referenced()
    colors = {}
    tot_src = tot_out = 0
    for rel in sorted(inst):
        n = inst[rel]
        p = src_path(rel)
        if not os.path.exists(p):
            print("WARNING: source missing, skipped:", rel)
            continue
        target = BUDGET.get(rel, BUDGET["_default"])
        if rel.endswith(".dae"):
            parts = load_dae(p)
            src_tris = sum(len(f) for _, f, _ in parts.values())
            total_v = sum(len(f) for _, f, _ in parts.values())
            out_tris = 0
            for name, (v, f, rgba) in parts.items():
                t = max(200, int(round(target * len(f) / max(total_v, 1)))) if src_tris > target else len(f)
                v2, f2 = decimate(v, f, t)
                op = os.path.join(OUT, rel + "." + name + ".stl")
                write_stl(op, v2, f2)
                colors[rel + "." + name] = rgba
                out_tris += len(f2)
                print("  %-34s %-10s %7d -> %6d  bounds(m) %s rgba %s" % (rel, name, len(f), len(f2), fmt_b(v2), ["%.2f" % c for c in rgba]))
            print("%-36s x%d  %7d -> %6d" % (rel, n, src_tris, out_tris))
        else:
            v, f = load_stl(p)
            src_tris = len(f)
            v2, f2 = decimate(v, f, target)
            out_tris = len(f2)
            write_stl(os.path.join(OUT, rel), v2, f2)
            print("%-36s x%d  %7d -> %6d  bounds %s" % (rel, n, src_tris, out_tris, fmt_b(v2)))
        tot_src += src_tris * n
        tot_out += out_tris * n
    json.dump(colors, open(os.path.join(OUT, "colors.json"), "w"), indent=1, sort_keys=True)
    print("TOTAL instanced triangles: source %d -> decimated %d (limit %d)" % (tot_src, tot_out, BUDGET["_total_instanced_max"]))
    if tot_out > BUDGET["_total_instanced_max"]:
        print("ERROR: over total budget"); sys.exit(1)


if __name__ == "__main__":
    main()
