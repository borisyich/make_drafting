"""Fillet detection utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Set

from afr3d.geom.primitives import PlanarFaceInfo, CylindricalFaceInfo, angle_between_normals_deg


@dataclass
class FilletCandidate:
    cyl_face: CylindricalFaceInfo
    neighbor_planes: List[PlanarFaceInfo]
    neighbor_cyls: List[CylindricalFaceInfo]
    kind: str
    plane_plane_angle_deg: Optional[float] = None


@dataclass
class BasicFillet:
    cyl_face: CylindricalFaceInfo
    kind: str
    plane_plane_angle_deg: float | None
    n_neighbor_planes: int
    n_neighbor_cyls: int


@dataclass
class GeometryMask:
    fillet_faces_all: Set[int]
    fillet_faces_basic: Set[int]
    non_fillet_cylindrical: Set[int]


def detect_fillet_candidates(
    partial_cyl_faces: List[CylindricalFaceInfo],
    planar_by_index: Dict[int, PlanarFaceInfo],
    cyl_by_index: Dict[int, CylindricalFaceInfo],
    adj: Dict[int, set[int]],
    min_plane_plane_angle_deg: float = 5.0,
    max_plane_plane_angle_deg: float = 175.0,
) -> List[FilletCandidate]:
    fillets: List[FilletCandidate] = []
    for cf in partial_cyl_faces:
        neighbors = adj.get(cf.face_index, set())

        neighbor_planes: List[PlanarFaceInfo] = []
        neighbor_cyls: List[CylindricalFaceInfo] = []
        for nfi in neighbors:
            if nfi in planar_by_index:
                neighbor_planes.append(planar_by_index[nfi])
            elif nfi in cyl_by_index:
                neighbor_cyls.append(cyl_by_index[nfi])

        kind = "unknown"
        plane_plane_angle_deg = None

        if len(neighbor_planes) >= 2:
            p1, p2 = neighbor_planes[0], neighbor_planes[1]
            ang = angle_between_normals_deg(p1.normal, p2.normal)
            if min_plane_plane_angle_deg < ang < max_plane_plane_angle_deg:
                kind = "plane-plane"
                plane_plane_angle_deg = ang
        elif len(neighbor_planes) == 1 and len(neighbor_cyls) >= 1:
            kind = "plane-cylinder"
        elif len(neighbor_planes) == 0 and len(neighbor_cyls) >= 2:
            kind = "cylinder-cylinder"

        fillets.append(
            FilletCandidate(
                cyl_face=cf,
                neighbor_planes=neighbor_planes,
                neighbor_cyls=neighbor_cyls,
                kind=kind,
                plane_plane_angle_deg=plane_plane_angle_deg,
            )
        )

    return fillets


def select_basic_fillets(
    fillet_candidates: List[FilletCandidate],
    max_plane_neighbors: int = 3,
    min_angle_deg: float = 10.0,
    max_angle_deg: float = 170.0,
) -> List[BasicFillet]:
    basic: List[BasicFillet] = []
    for fc in fillet_candidates:
        if fc.kind not in ("plane-plane", "plane-cylinder"):
            continue

        n_pl = len(fc.neighbor_planes)
        n_cy = len(fc.neighbor_cyls)
        if n_pl > max_plane_neighbors:
            continue

        if fc.kind == "plane-plane":
            ang = fc.plane_plane_angle_deg
            if ang is None or not (min_angle_deg <= ang <= max_angle_deg):
                continue

        basic.append(
            BasicFillet(
                cyl_face=fc.cyl_face,
                kind=fc.kind,
                plane_plane_angle_deg=fc.plane_plane_angle_deg,
                n_neighbor_planes=n_pl,
                n_neighbor_cyls=n_cy,
            )
        )

    return basic


def build_geometry_mask(
    full_cyl_faces: List[CylindricalFaceInfo],
    partial_cyl_faces: List[CylindricalFaceInfo],
    fillet_candidates: List[FilletCandidate],
    basic_fillets: List[BasicFillet],
) -> GeometryMask:
    all_cyl_ids = {cf.face_index for cf in (full_cyl_faces + partial_cyl_faces)}
    all_fillet_ids = {fc.cyl_face.face_index for fc in fillet_candidates}
    basic_fillet_ids = {bf.cyl_face.face_index for bf in basic_fillets}
    non_fillet_cyl = all_cyl_ids - all_fillet_ids

    return GeometryMask(
        fillet_faces_all=all_fillet_ids,
        fillet_faces_basic=basic_fillet_ids,
        non_fillet_cylindrical=non_fillet_cyl,
    )


__all__ = [
    "BasicFillet",
    "FilletCandidate",
    "GeometryMask",
    "build_geometry_mask",
    "detect_fillet_candidates",
    "select_basic_fillets",
]
