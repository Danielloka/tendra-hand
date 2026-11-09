"""Software/headless renderer for CadQuery parts and STL files.

Primary backend: three.js in headless Chromium (SwiftShader WebGL): z-buffered, lit, anti-aliased.
Fallback: pure-numpy z-buffer rasteriser (used automatically if node/chromium fails).

    from tools.render import render, tessellate
    render({"a": wp1, "b": "part.stl"}, "out.png", views=("iso","front","side","top"),
           colors={"a": "#d04040"}, size=(1600,1200), title="demo")

Views: names "iso","iso2","front"(from -Y, palm side),"back","side"(from +X),"left","top"(from +Z),
"bottom", or a custom (elev_deg, azim_deg) tuple / (label, elev, azim). Z is up; azim 0 looks from -Y,
azim 90 from +X.
CLI: python render.py a.stl b.step -o out.png [--views iso,front] [--size 1600x1200] [--title T]
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

NAMED_VIEWS = {
    "iso": (25, 35), "iso2": (25, 215), "front": (0, 0), "back": (0, 180), "side": (0, 90),
    "left": (0, -90), "top": (90, 0), "bottom": (-90, 0),
}
PALETTE = ["#4f86c6", "#e08a3c", "#5bab6b", "#c25b6b", "#8e6fbf", "#c9ae43", "#3fa3a8", "#a0754d",
           "#7a8aa0", "#d46fa4", "#6c9a3d", "#b0603a"]


def _is_cq(o):
    return hasattr(o, "val") or hasattr(o, "tessellate") or hasattr(o, "wrapped")


def tessellate(obj, tol=0.15, ang=0.3):
    """-> (verts (N,3) float64, faces (M,3) int) for a CadQuery Workplane/Shape/Compound or STL/STEP path."""
    import trimesh
    if isinstance(obj, (str, os.PathLike)):
        p = str(obj)
        if p.lower().endswith((".step", ".stp")):
            import cadquery as cq
            obj = cq.importers.importStep(p)
        else:
            m = trimesh.load(p, force="mesh")
            return np.asarray(m.vertices, float), np.asarray(m.faces, int)
    if isinstance(obj, trimesh.Trimesh):
        return np.asarray(obj.vertices, float), np.asarray(obj.faces, int)
    shapes = obj.vals() if hasattr(obj, "vals") else [obj]
    V, F, off = [], [], 0
    for s in shapes:
        if hasattr(s, "toCompound") is False and hasattr(s, "val"):
            s = s.val()
        v, t = s.tessellate(tol, ang)
        if not v:
            continue
        V.append(np.array([[p.x, p.y, p.z] for p in v], float))
        F.append(np.array(t, int) + off)
        off += len(v)
    if not V:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    return np.vstack(V), np.vstack(F)


def _norm_views(views):
    out = []
    for v in views:
        if isinstance(v, str):
            e, a = NAMED_VIEWS[v]
            out.append((v, e, a))
        elif len(v) == 2:
            out.append((f"el {v[0]:g} az {v[1]:g}", v[0], v[1]))
        else:
            out.append(tuple(v))
    return out


def _hex(c):
    if isinstance(c, (tuple, list)):
        return "#%02x%02x%02x" % tuple(int(x * 255) if max(c) <= 1 else int(x) for x in c[:3])
    return c


def _darken(h, f=0.45):
    h = h.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
    return "#%02x%02x%02x" % (int(r * f), int(g * f), int(b * f))


def render(parts, out_png, views=("iso", "front", "side", "top"), colors=None, size=(1600, 1200),
           title=None, tol=0.15, ang=0.3, edges=True, axes=True, legend=None, bg="#f4f5f7",
           backend="auto"):
    """Render a dict name -> shape/STL path into one PNG (grid of views). Returns the out path."""
    if not isinstance(parts, dict):
        parts = {f"part{i}": p for i, p in enumerate(parts)}
    colors = colors or {}
    vws = _norm_views(views)
    tris, meta, off = [], [], 0
    for i, (name, obj) in enumerate(parts.items()):
        v, f = tessellate(obj, tol, ang)
        if len(f) == 0:
            continue
        soup = v[f].astype(np.float32)  # (M,3,3)
        col = _hex(colors.get(name, PALETTE[i % len(PALETTE)]))
        meta.append({"name": name, "color": col, "edgeColor": _darken(col), "offset": off, "count": len(f)})
        tris.append(soup.reshape(-1))
        off += len(f) * 9
    if not tris:
        raise ValueError("nothing to render")
    soup = np.concatenate(tris)
    out_png = str(out_png)
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    legend = len(meta) <= 25 if legend is None else legend
    if backend in ("auto", "gl") and (HERE / "node_modules" / "three").exists():
        with tempfile.TemporaryDirectory() as td:
            binp = Path(td) / "tris.bin"
            soup.tofile(binp)
            job = {"bin": str(binp), "out": out_png, "size": list(size), "views": [dict(label=l, elev=e, azim=a) for l, e, a in vws],
                   "title": title, "axes": axes, "edges": edges, "bg": bg, "parts": meta, "legend": legend}
            jp = Path(td) / "job.json"
            jp.write_text(json.dumps(job))
            r = subprocess.run(["node", str(HERE / "render_gl.js"), str(jp)], capture_output=True, text=True,
                               timeout=300, env={**os.environ, "NODE_PATH": os.environ.get("NODE_PATH", "") + ":/opt/node-tools/node_modules"})
        if r.returncode == 0 and os.path.exists(out_png):
            return out_png
        print("GL backend failed, falling back to numpy:", r.stderr[-500:], file=sys.stderr)
    return _render_numpy(soup, meta, vws, out_png, size, title, bg)


def _render_numpy(soup, meta, vws, out_png, size, title, bg):
    from PIL import Image, ImageDraw
    tri = soup.reshape(-1, 3, 3).astype(np.float64)
    cols = np.zeros((len(tri), 3))
    for m in meta:
        c = _hex(m["color"]).lstrip("#")
        cols[m["offset"] // 9: m["offset"] // 9 + m["count"]] = [int(c[i:i + 2], 16) for i in (0, 2, 4)]
    n = len(vws)
    nc = 1 if n <= 1 else (n if n <= 3 else int(np.ceil(np.sqrt(n))))
    nr = int(np.ceil(n / nc))
    W, H = size
    cw, ch = W // nc, H // nr
    S = 2  # supersample
    canvas = Image.new("RGB", (W, H), bg)
    mn, mx = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
    ctr = (mn + mx) / 2
    for i, (label, el, az) in enumerate(vws):
        e, a = np.radians(el), np.radians(az)
        d = np.array([np.sin(a) * np.cos(e), -np.cos(a) * np.cos(e), np.sin(e)])
        up0 = np.array([0, 0, 1.0]) if abs(np.sin(e)) < 0.999 else np.array([0, np.sign(np.sin(e)), 0])
        rt = np.cross(up0, d); rt /= np.linalg.norm(rt)
        up = np.cross(d, rt)
        P = np.stack([(tri - ctr) @ rt, (tri - ctr) @ up, (tri - ctr) @ d], -1)  # x right, y up, z toward cam
        ext = np.abs(P[..., :2]).max()
        sc = min(cw * S, ch * S) / (2 * ext * 1.12)
        X = P[..., 0] * sc + cw * S / 2
        Y = ch * S / 2 - P[..., 1] * sc
        Z = P[..., 2]
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nl = np.linalg.norm(nrm, axis=1, keepdims=True); nl[nl == 0] = 1
        nrm = nrm / nl
        nrm = nrm * np.where((nrm @ d) < 0, -1, 1)[:, None]
        L = d * 0.6 - rt * 0.5 + up * 0.6; L /= np.linalg.norm(L)
        sh = 0.35 + 0.65 * np.clip(nrm @ L, 0, 1)
        img = np.zeros((ch * S, cw * S, 3)); img[:] = [int(bg[k:k + 2], 16) for k in (1, 3, 5)]
        zb = np.full((ch * S, cw * S), -np.inf)
        for t in range(len(tri)):
            x, y, z = X[t], Y[t], Z[t]
            x0, x1 = int(max(np.floor(x.min()), 0)), int(min(np.ceil(x.max()), cw * S - 1))
            y0, y1 = int(max(np.floor(y.min()), 0)), int(min(np.ceil(y.max()), ch * S - 1))
            if x1 < x0 or y1 < y0:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + .5, np.arange(y0, y1 + 1) + .5)
            den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
            if abs(den) < 1e-12:
                continue
            w0 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
            w1 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
            w2 = 1 - w0 - w1
            m = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
            zz = w0 * z[0] + w1 * z[1] + w2 * z[2]
            sub = zb[y0:y1 + 1, x0:x1 + 1]
            m &= zz > sub
            sub[m] = zz[m]
            img[y0:y1 + 1, x0:x1 + 1][m] = cols[t] * sh[t]
        im = Image.fromarray(img.clip(0, 255).astype(np.uint8)).resize((cw, ch), Image.LANCZOS)
        canvas.paste(im, ((i % nc) * cw, (i // nc) * ch))
        ImageDraw.Draw(canvas).text(((i % nc) * cw + 10, (i // nc) * ch + 8), label, fill="#444")
    if title:
        ImageDraw.Draw(canvas).text((W - 200, 8), title, fill="#111")
    canvas.save(out_png)
    return out_png


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--views", default="iso,front,side,top")
    ap.add_argument("--size", default="1600x1200")
    ap.add_argument("--title")
    ap.add_argument("--backend", default="auto")
    a = ap.parse_args(argv)
    parts = {Path(f).stem: f for f in a.files}
    views = [tuple(map(float, v.split(":"))) if ":" in v else v for v in a.views.split(",")]
    w, h = map(int, a.size.split("x"))
    print(render(parts, a.out, views=views, size=(w, h), title=a.title, backend=a.backend))


if __name__ == "__main__":
    main()
