"""High-level feature extraction pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from afr3d.features.cylinders import (
    CylindricalFeature,
    CylindricalSegmentGroup,
    group_cylindrical_segments,
    group_full_cylinders_to_features,
    split_cyl_features_into_holes_and_bosses,
)
from afr3d.features.fillets import BasicFillet, FilletCandidate, build_geometry_mask, detect_fillet_candidates, select_basic_fillets
from afr3d.features.holes import HoleAFR, detect_holes_afr
from afr3d.geom.primitives import CylindricalFaceInfo, PlanarFaceInfo
from afr3d.topo.topology import (
    build_face_adjacency,
    collect_faces_and_map,
    extract_cylindrical_faces,
    extract_conical_faces,
    extract_planar_faces,
)


@dataclass
class PartAnalysis:
    faces: list
    adjacency: dict[int, set[int]]
    planar_faces: list[PlanarFaceInfo]
    full_cyl_faces: list[CylindricalFaceInfo]
    partial_cyl_faces: list[CylindricalFaceInfo]
    cyl_features: list[CylindricalFeature]
    hole_features: list[CylindricalFeature]
    boss_features: list[CylindricalFeature]
    fillet_candidates: list[FilletCandidate]
    basic_fillets: list[BasicFillet]


def extract_features_from_shape(
    shape,
    full_circle_tol_deg: float = 5.0,
    axis_ang_tol: float = 1e-3,
    axis_pos_tol: float = 1e-2,
    radius_tol: float = 1e-3,
):
    planar_faces = extract_planar_faces(shape)
    full_cyl_faces, partial_cyl_faces = extract_cylindrical_faces(shape, full_circle_tol_deg=full_circle_tol_deg)

    cyl_features = group_full_cylinders_to_features(
        full_cyl_faces, axis_ang_tol=axis_ang_tol, axis_pos_tol=axis_pos_tol, radius_tol=radius_tol
    )
    cyl_segment_groups = group_cylindrical_segments(
        partial_cyl_faces, axis_ang_tol=axis_ang_tol, axis_pos_tol=axis_pos_tol, radius_tol=radius_tol
    )
    hole_features, boss_features, ambiguous_features = split_cyl_features_into_holes_and_bosses(cyl_features)

    return (
        planar_faces,
        full_cyl_faces,
        partial_cyl_faces,
        cyl_features,
        cyl_segment_groups,
        hole_features,
        boss_features,
        ambiguous_features,
    )


def analyze_shape_for_fillets(shape):
    faces, face_map = collect_faces_and_map(shape)
    adj = build_face_adjacency(shape, face_map)
    (
        planar_faces,
        full_cyl_faces,
        partial_cyl_faces,
        cyl_features,
        cyl_segment_groups,
        hole_features,
        boss_features,
        ambiguous_features,
    ) = extract_features_from_shape(shape)

    planar_by_index = {pf.face_index: pf for pf in planar_faces}
    cyl_by_index = {cf.face_index: cf for cf in (full_cyl_faces + partial_cyl_faces)}

    fillet_candidates = detect_fillet_candidates(
        partial_cyl_faces=partial_cyl_faces,
        planar_by_index=planar_by_index,
        cyl_by_index=cyl_by_index,
        adj=adj,
    )
    basic_fillets = select_basic_fillets(fillet_candidates)
    geom_mask = build_geometry_mask(
        full_cyl_faces=full_cyl_faces,
        partial_cyl_faces=partial_cyl_faces,
        fillet_candidates=fillet_candidates,
        basic_fillets=basic_fillets,
    )

    return (
        planar_faces,
        full_cyl_faces,
        partial_cyl_faces,
        fillet_candidates,
        basic_fillets,
        geom_mask,
        adj,
    )


def analyze_part(shape) -> PartAnalysis:
    faces, face_map = collect_faces_and_map(shape)
    adj = build_face_adjacency(shape, face_map)

    (
        planar_faces,
        full_cyl_faces,
        partial_cyl_faces,
        cyl_features,
        cyl_segment_groups,
        hole_features,
        boss_features,
        ambiguous_features,
    ) = extract_features_from_shape(shape)

    hole_features, boss_features, ambiguous = split_cyl_features_into_holes_and_bosses(cyl_features)
    fillet_candidates = detect_fillet_candidates(
        partial_cyl_faces,
        planar_by_index={pf.face_index: pf for pf in planar_faces},
        cyl_by_index={cf.face_index: cf for cf in full_cyl_faces + partial_cyl_faces},
        adj=adj,
    )
    basic_fillets = select_basic_fillets(fillet_candidates)

    return PartAnalysis(
        faces=faces,
        adjacency=adj,
        planar_faces=planar_faces,
        full_cyl_faces=full_cyl_faces,
        partial_cyl_faces=partial_cyl_faces,
        cyl_features=cyl_features,
        hole_features=hole_features,
        boss_features=boss_features,
        fillet_candidates=fillet_candidates,
        basic_fillets=basic_fillets,
    )


def detect_holes(shape) -> List[HoleAFR]:
    (
        planar_faces,
        full_cyl_faces,
        partial_cyl_faces,
        cyl_features,
        cyl_segment_groups,
        hole_features,
        boss_features,
        ambiguous_features,
    ) = extract_features_from_shape(shape)

    faces, face_map = collect_faces_and_map(shape)
    adj = build_face_adjacency(shape, face_map)
    conical_faces = extract_conical_faces(shape)

    return detect_holes_afr(
        shape,
        hole_features,
        planar_faces,
        adj,
        conical_faces,
        chamfer_max_length=2.0,
    )


__all__ = [
    "BasicFillet",
    "CylindricalFeature",
    "CylindricalSegmentGroup",
    "FilletCandidate",
    "HoleAFR",
    "PartAnalysis",
    "analyze_part",
    "analyze_shape_for_fillets",
    "detect_holes",
    "detect_holes_afr",
    "extract_features_from_shape",
    "group_cylindrical_segments",
    "group_full_cylinders_to_features",
    "split_cyl_features_into_holes_and_bosses",
]
