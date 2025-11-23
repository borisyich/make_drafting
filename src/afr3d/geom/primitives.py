"""Geometric primitives and helpers used across feature detectors."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple

from OCC.Core.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane
from OCC.Core.TopoDS import TopoDS_Face
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepGProp import brepgprop_SurfaceProperties
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import TopAbs_FORWARD, TopAbs_REVERSED
from OCC.Core.gp import gp_Dir, gp_Pnt


@dataclass
class PlanarFaceInfo:
    face_index: int
    point: Tuple[float, float, float]
    normal: Tuple[float, float, float]
    area: float
    orientation: str

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        px, py, pz = self.point
        nx, ny, nz = self.normal
        return (
            "PlanarFace(face_index={self.face_index}, "
            f"point=({px:.3f}, {py:.3f}, {pz:.3f}), "
            f"normal=({nx:.3f}, {ny:.3f}, {nz:.3f}), "
            f"area={self.area:.3f}, orientation={self.orientation})"
        )


@dataclass
class CylindricalFaceInfo:
    face_index: int
    axis_origin: Tuple[float, float, float]
    axis_dir: Tuple[float, float, float]
    radius: float
    u_first: float
    u_last: float
    u_span: float
    v_first: float
    v_last: float
    v_span: float
    is_u_closed: bool
    is_v_closed: bool
    orientation: str
    area: float
    mid_point: Tuple[float, float, float]
    topods_face: TopoDS_Face

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        ox, oy, oz = self.axis_origin
        dx, dy, dz = self.axis_dir
        mx, my, mz = self.mid_point
        return (
            f"CylFace(idx={self.face_index}, R={self.radius:.3f}, "
            f"axis_origin=({ox:.3f},{oy:.3f},{oz:.3f}), "
            f"axis_dir=({dx:.3f},{dy:.3f},{dz:.3f}), "
            f"u_span={self.u_span:.3f}, v_span={self.v_span:.3f}, "
            f"area={self.area:.3f}, mid=({mx:.3f},{my:.3f},{mz:.3f}), "
            f"u_closed={self.is_u_closed}, v_closed={self.is_v_closed}, "
            f"orientation={self.orientation}), "
        )

@dataclass
class CylindricalFeature:
    axis_origin: Tuple[float, float, float]
    axis_dir: Tuple[float, float, float]
    radius: float
    face_indices: List[int] = field(default_factory=list)
    faces: List[CylindricalFaceInfo] = field(default_factory=list)

    def __repr__(self):
        ox, oy, oz = self.axis_origin
        dx, dy, dz = self.axis_dir
        return (f"CylFeature(R={self.radius:.3f}, "
                f"axis_origin=({ox:.3f},{oy:.3f},{oz:.3f}), "
                f"axis_dir=({dx:.3f},{dy:.3f},{dz:.3f}), "
                f"faces={self.face_indices})")

@dataclass
class ConicalFaceInfo:
    face_index: int
    apex: Tuple[float, float, float]
    axis_dir: Tuple[float, float, float]
    semi_angle_deg: float
    ref_radius: float
    u_span: float
    v_span: float
    length: float


def gp_dir_to_tuple(d: gp_Dir) -> Tuple[float, float, float]:
    return (d.X(), d.Y(), d.Z())


def gp_pnt_to_tuple(p: gp_Pnt) -> Tuple[float, float, float]:
    return (p.X(), p.Y(), p.Z())


def normalize_vec(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (x / n, y / n, z / n)


def almost_equal(a: float, b: float, tol: float = 1e-4) -> bool:
    return abs(a - b) <= tol


def vec_dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_norm(a: Tuple[float, float, float]) -> float:
    return math.sqrt(vec_dot(a, a))


def vec_angle(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    na = vec_norm(a)
    nb = vec_norm(b)
    if na < 1e-12 or nb < 1e-12:
        return math.pi
    dot = vec_dot(a, b) / (na * nb)
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot)


def vec_almost_equal(
    v1: Tuple[float, float, float], v2: Tuple[float, float, float], ang_tol: float = 1e-3
) -> bool:
    x1, y1, z1 = normalize_vec(v1)
    x2, y2, z2 = normalize_vec(v2)
    dot = max(-1.0, min(1.0, x1 * x2 + y1 * y2 + z1 * z2))
    angle = math.acos(dot)
    return angle <= ang_tol or abs(angle - math.pi) <= ang_tol


def dist_between_axes(origin1: Tuple[float, float, float], dir1: Tuple[float, float, float], origin2):
    import numpy as np

    p1 = np.array(origin1, dtype=float)
    d1 = np.array(dir1, dtype=float)
    p2 = np.array(origin2, dtype=float)
    d2 = np.array(dir1, dtype=float)

    v = p2 - p1
    cross = np.cross(d1, d2)
    n2 = np.dot(cross, cross)
    if n2 < 1e-12:
        proj = np.dot(v, d1) / np.dot(d1, d1)
        perp = v - proj * d1
        return float(np.linalg.norm(perp))
    return float(abs(np.dot(v, cross)) / math.sqrt(n2))


def angle_between_normals_deg(
    n1: Tuple[float, float, float], n2: Tuple[float, float, float]
) -> float:
    x1, y1, z1 = normalize_vec(n1)
    x2, y2, z2 = normalize_vec(n2)
    dot = max(-1.0, min(1.0, x1 * x2 + y1 * y2 + z1 * z2))
    return math.degrees(math.acos(dot))


__all__ = [
    "PlanarFaceInfo",
    "CylindricalFaceInfo",
    "ConicalFaceInfo",
    "gp_dir_to_tuple",
    "gp_pnt_to_tuple",
    "normalize_vec",
    "almost_equal",
    "vec_dot",
    "vec_norm",
    "vec_angle",
    "vec_almost_equal",
    "dist_between_axes",
    "angle_between_normals_deg",
]
