"""Placement and strand/sheath geometry helpers (mm, degrees).

    rotate_about(shape, point, direction, deg)   rotate a Workplane/Shape about an arbitrary axis
    translate(shape, v) ; place(shape, origin=(0,0,0), axis=None, deg=0)  rotate about origin axis, then move
    chain_rotations(shape, [(point, dir, deg), ...])  apply several joint rotations (distal-first order given)
    polyline_length(pts)
    arc_on_drum(centre, axis, radius, p_from, p_to, sense=None) -> arc length wrapped on a drum
    tangent_point(centre, axis, radius, ext_point, side=+1) -> tangent point on the drum circle
    strand_length(segments) : mix of ("line", p, q) and ("arc", centre, axis, r, p_from, p_to[, sense])
    min_bend_radius(pts, smooth=False) -> (min_radius, index) ; check_sheath(pts, min_r=15) -> dict
    spline_through(pts, n=200) -> dense polyline of a smooth spline (use before min_bend_radius)
"""
from __future__ import annotations

import numpy as np


def _v(p):
    return np.asarray(p, float)


def rotate_about(shape, point, direction, deg):
    """Rotate about the axis through `point` with `direction` by `deg` degrees (right-hand rule)."""
    p, d = _v(point), _v(direction)
    return shape.rotate(tuple(p), tuple(p + d), deg)


def translate(shape, v):
    return shape.translate(tuple(_v(v)))


def place(shape, origin=(0, 0, 0), axis=None, deg=0.0):
    """Rotate about an axis through the shape's frame origin (0,0,0) by deg, then move to `origin`."""
    if axis is not None and deg:
        shape = shape.rotate((0, 0, 0), tuple(_v(axis)), deg)
    return shape.translate(tuple(_v(origin)))


def chain_rotations(shape, joints):
    """joints = [(point, direction, deg), ...] applied in the given order (put the most distal joint first)."""
    for p, d, a in joints:
        shape = rotate_about(shape, p, d, a)
    return shape


def rot_matrix(direction, deg):
    d = _v(direction) / np.linalg.norm(direction)
    a = np.radians(deg)
    K = np.array([[0, -d[2], d[1]], [d[2], 0, -d[0]], [-d[1], d[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def rotate_point(pt, point, direction, deg):
    """Rotate a point (not a solid) about an axis; useful for moving anchor/interface points with the part."""
    return _v(point) + rot_matrix(direction, deg) @ (_v(pt) - _v(point))


def polyline_length(pts):
    p = _v(pts)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def _basis(axis):
    a = _v(axis) / np.linalg.norm(axis)
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, t); u /= np.linalg.norm(u)
    return a, u, np.cross(a, u)


def arc_on_drum(centre, axis, radius, p_from, p_to, sense=None):
    """Arc length of strand wrapped on a drum between two points on (or projected onto) its circle.
    sense = +1 wraps counter-clockwise about `axis` (right-hand rule) from p_from to p_to, -1 clockwise,
    None = the shorter way. Returns radius * wrap angle."""
    a, u, v = _basis(axis)
    c = _v(centre)
    def ang(p):
        r = _v(p) - c
        return np.arctan2(r @ v, r @ u)
    d = ang(p_to) - ang(p_from)
    if sense is None:
        d = (d + np.pi) % (2 * np.pi) - np.pi
        return float(radius * abs(d))
    d = d % (2 * np.pi) if sense > 0 else (-d) % (2 * np.pi)
    return float(radius * d)


def tangent_point(centre, axis, radius, ext_point, side=+1):
    """Tangent point on a drum circle for a straight strand leaving from `ext_point` (projected into the drum
    plane). side=+1/-1 picks the left/right tangent seen along +axis. Returns a 3D point in the drum plane."""
    a, u, v = _basis(axis)
    c = _v(centre)
    r = _v(ext_point) - c
    x, y = r @ u, r @ v
    dist = np.hypot(x, y)
    if dist <= radius:
        raise ValueError("point is inside the drum")
    phi = np.arctan2(y, x)
    off = np.arccos(radius / dist)
    th = phi + side * off
    return c + radius * (np.cos(th) * u + np.sin(th) * v)


def strand_length(segments):
    """Total length of mixed segments: ("line", p, q) or ("arc", centre, axis, r, p_from, p_to[, sense])."""
    tot = 0.0
    for s in segments:
        if s[0] == "line":
            tot += float(np.linalg.norm(_v(s[2]) - _v(s[1])))
        else:
            tot += arc_on_drum(*s[1:])
    return tot


def spline_through(pts, n=200, smooth=0.0):
    from scipy.interpolate import splev, splprep
    p = _v(pts)
    k = min(3, len(p) - 1)
    tck, _ = splprep(p.T, s=smooth, k=k)
    return np.array(splev(np.linspace(0, 1, n), tck)).T


def curvature_radii(pts):
    """Radius of curvature at each interior vertex (circumradius of consecutive triples); inf if collinear."""
    p = _v(pts)
    a, b, c = p[:-2], p[1:-1], p[2:]
    ab, bc, ca = np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)
    cross = np.linalg.norm(np.cross(b - a, c - b), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = ab * bc * ca / (2 * cross)
    return np.where(cross < 1e-12, np.inf, r)


def min_bend_radius(pts, smooth=False, n=300):
    """Minimum radius of curvature (mm) of a path and the index where it occurs. For a coarse polyline with sharp
    corners the result is the circumradius at each vertex (corners of a polyline are not real bends: use
    smooth=True to fit a spline through the waypoints first)."""
    p = spline_through(pts, n) if smooth else _v(pts)
    r = curvature_radii(p)
    i = int(np.argmin(r))
    return float(r[i]), i


def check_sheath(pts, min_r=15.0, smooth=True):
    """dict(min_radius, ok, at_point, length). smooth=True fits a spline through the waypoints."""
    r, i = min_bend_radius(pts, smooth=smooth)
    q = spline_through(pts) if smooth else _v(pts)
    return dict(min_radius=r, ok=bool(r >= min_r), at_point=q[min(i + 1, len(q) - 1)].tolist(),
                length=polyline_length(q))
