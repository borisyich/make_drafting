"""Hole detection and AFR abstractions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Literal, Tuple

from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.TopAbs import TopAbs_IN, TopAbs_ON, TopAbs_OUT
from OCC.Core.gp import gp_Pnt

from afr3d.geom.primitives import (
    PlanarFaceInfo,
    ConicalFaceInfo,
    CylindricalFaceInfo,
    dist_between_axes,
    vec_angle,
    vec_dot,
    vec_norm,
)
from afr3d.topo.topology import get_part_max_dim


HoleKind = Literal["through", "blind", "unknown"]
HoleGeometryType = Literal["circular_simple", "circular_stepped", "non_circular", "unknown"]


@dataclass
class HoleSegment:
    kind: Literal["cyl", "cone"]
    radius: float | None
    length: float
    face_indices: List[int]


@dataclass
class ChamferInfo:
    face_index: int
    length: float
    semi_angle_deg: float
    side: str


@dataclass
class HoleAFR:
    id: int
    geometry_type: HoleGeometryType
    kind: HoleKind
    axis_origin: Tuple[float, float, float] | None
    axis_dir: Tuple[float, float, float] | None
    nominal_radius: float | None
    segments: List[HoleSegment]
    side_face_indices: List[int]
    opening_faces: List[int]
    bottom_faces: List[int]
    chamfers: List[ChamferInfo]


def cluster_hole_features_by_axis(
    hole_features: List["CylindricalFeature"],
    ang_tol_rad: float = math.radians(1.0),
    axis_dist_tol: float = 0.5,
):
    clusters: List[List["CylindricalFeature"]] = []
    for feat in hole_features:
        added = False
        for cluster in clusters:
            ref = cluster[0]
            ang = vec_angle(feat.axis_dir, ref.axis_dir)
            if ang > ang_tol_rad and abs(ang - math.pi) > ang_tol_rad:
                continue
            dist = dist_between_axes(feat.axis_origin, feat.axis_dir, ref.axis_origin)
            if dist > axis_dist_tol:
                continue
            cluster.append(feat)
            added = True
            break
        if not added:
            clusters.append([feat])
    return clusters


def classify_hole_through_blind(
    shape, axis_origin: Tuple[float, float, float], axis_dir: Tuple[float, float, float]
) -> HoleKind:
    L = get_part_max_dim(shape) * 1.2
    ox, oy, oz = axis_origin
    dx, dy, dz = axis_dir
    p_plus = gp_Pnt(ox + dx * L, oy + dy * L, oz + dz * L)
    p_minus = gp_Pnt(ox - dx * L, oy - dy * L, oz - dz * L)

    classifier = BRepClass3d_SolidClassifier(shape)
    classifier.Perform(p_plus, 1e-3)
    st_plus = classifier.State()
    classifier.Perform(p_minus, 1e-3)
    st_minus = classifier.State()

    plus_out = st_plus in (TopAbs_OUT, TopAbs_ON)
    minus_out = st_minus in (TopAbs_OUT, TopAbs_ON)
    plus_in = st_plus == TopAbs_IN
    minus_in = st_minus == TopAbs_IN

    if plus_out and minus_out:
        return "through"
    if (plus_in and minus_out) or (minus_in and plus_out):
        return "blind"
    return "unknown"


def detect_holes_afr(
    shape,
    hole_features: List["CylindricalFeature"],
    planar_faces: List[PlanarFaceInfo],
    adj: Dict[int, set[int]],
    conical_faces: List[ConicalFaceInfo],
    chamfer_max_length: float = 2.0,
) -> List[HoleAFR]:
    planar_by_index = {pf.face_index: pf for pf in planar_faces}
    conical_by_index = {cf.face_index: cf for cf in conical_faces}
    clusters = cluster_hole_features_by_axis(hole_features)

    holes_afr: List[HoleAFR] = []
    for hid, cluster in enumerate(clusters):
        axis_dir = cluster[0].axis_dir
        ax = sum(f.axis_origin[0] for f in cluster) / len(cluster)
        ay = sum(f.axis_origin[1] for f in cluster) / len(cluster)
        az = sum(f.axis_origin[2] for f in cluster) / len(cluster)
        axis_origin = (ax, ay, az)

        cyl_faces = [cf for feat in cluster for cf in feat.faces]
        cyl_face_indices = [cf.face_index for cf in cyl_faces]

        radii = sorted({round(cf.radius, 4) for cf in cyl_faces})
        if len(radii) == 1:
            geometry_type: HoleGeometryType = "circular_simple"
            nominal_radius = radii[0]
        else:
            geometry_type = "circular_stepped"
            nominal_radius = max(radii)

        hole_kind = classify_hole_through_blind(shape, axis_origin, axis_dir)

        neighbor_indices = set()
        for fi in cyl_face_indices:
            neighbor_indices.update(adj.get(fi, set()))

        opening_faces: List[int] = []
        bottom_faces: List[int] = []
        chamfers: List[ChamferInfo] = []
        segments: List[HoleSegment] = []

        for nfi in neighbor_indices:
            if nfi in conical_by_index:
                cone = conical_by_index[nfi]
                dot = abs(vec_dot(cone.axis_dir, axis_dir) / (vec_norm(cone.axis_dir) * vec_norm(axis_dir)))
                if dot < 0.95:
                    continue
                if cone.length <= chamfer_max_length:
                    chamfers.append(
                        ChamferInfo(
                            face_index=nfi,
                            length=cone.length,
                            semi_angle_deg=cone.semi_angle_deg,
                            side="unknown",
                        )
                    )
                else:
                    segments.append(
                        HoleSegment(
                            kind="cone",
                            radius=cone.ref_radius,
                            length=cone.length,
                            face_indices=[nfi],
                        )
                    )
                    bottom_faces.append(nfi)

        if bottom_faces and hole_kind == "through":
            hole_kind = "blind"

        for nfi in neighbor_indices:
            if nfi in planar_by_index:
                pf = planar_by_index[nfi]
                ndot = abs(vec_dot(pf.normal, axis_dir) / (vec_norm(pf.normal) * vec_norm(axis_dir)))
                if ndot > 0.95:
                    opening_faces.append(nfi)

        for r in radii:
            faces_r = [cf for cf in cyl_faces if abs(cf.radius - r) < 1e-4]
            if not faces_r:
                continue
            v_min = min(cf.v_first for cf in faces_r)
            v_max = max(cf.v_last for cf in faces_r)
            length = abs(v_max - v_min)

            segments.append(
                HoleSegment(
                    kind="cyl",
                    radius=r,
                    length=abs(length),
                    face_indices=[cf.face_index for cf in faces_r],
                )
            )

        holes_afr.append(
            HoleAFR(
                id=hid,
                geometry_type=geometry_type,
                kind=hole_kind,
                axis_origin=axis_origin,
                axis_dir=axis_dir,
                nominal_radius=nominal_radius,
                segments=segments,
                side_face_indices=cyl_face_indices,
                opening_faces=opening_faces,
                bottom_faces=bottom_faces,
                chamfers=chamfers,
            )
        )

    return holes_afr


__all__ = [
    "ChamferInfo",
    "HoleAFR",
    "HoleGeometryType",
    "HoleKind",
    "HoleSegment",
    "classify_hole_through_blind",
    "cluster_hole_features_by_axis",
    "detect_holes_afr",
]
