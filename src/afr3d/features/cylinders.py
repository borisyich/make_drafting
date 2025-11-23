"""Detection and grouping of cylindrical faces."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from afr3d.geom.primitives import (
    CylindricalFaceInfo,
    almost_equal,
    dist_between_axes,
    vec_almost_equal,
)


@dataclass
class CylindricalFeature:
    axis_origin: Tuple[float, float, float]
    axis_dir: Tuple[float, float, float]
    radius: float
    face_indices: List[int] = field(default_factory=list)
    faces: List[CylindricalFaceInfo] = field(default_factory=list)

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        ox, oy, oz = self.axis_origin
        dx, dy, dz = self.axis_dir
        return (
            f"CylFeature(R={self.radius:.3f}, axis_origin=({ox:.3f},{oy:.3f},{oz:.3f}), "
            f"axis_dir=({dx:.3f},{dy:.3f},{dz:.3f}), faces={self.face_indices})"
        )


@dataclass
class CylindricalSegmentGroup:
    axis_origin: Tuple[float, float, float]
    axis_dir: Tuple[float, float, float]
    radius: float
    face_indices: List[int] = field(default_factory=list)
    u_spans: List[float] = field(default_factory=list)
    v_spans: List[float] = field(default_factory=list)

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        ox, oy, oz = self.axis_origin
        dx, dy, dz = self.axis_dir
        return (
            f"CylSegmentGroup(R={self.radius:.3f}, axis_origin=({ox:.3f},{oy:.3f},{oz:.3f}), "
            f"axis_dir=({dx:.3f},{dy:.3f},{dz:.3f}), faces={self.face_indices})"
        )


def group_cylindrical_segments(
    partial_faces: List[CylindricalFaceInfo],
    axis_ang_tol: float = 1e-3,
    axis_pos_tol: float = 1e-2,
    radius_tol: float = 1e-3,
) -> List[CylindricalSegmentGroup]:
    groups: List[CylindricalSegmentGroup] = []

    for cf in partial_faces:
        assigned = False
        for grp in groups:
            if not almost_equal(grp.radius, cf.radius, tol=radius_tol):
                continue
            if not vec_almost_equal(grp.axis_dir, cf.axis_dir, ang_tol=axis_ang_tol):
                continue
            dist = dist_between_axes(grp.axis_origin, grp.axis_dir, cf.axis_origin)
            if dist > axis_pos_tol:
                continue

            grp.face_indices.append(cf.face_index)
            grp.u_spans.append(cf.u_span)
            grp.v_spans.append(cf.v_span)
            assigned = True
            break

        if not assigned:
            groups.append(
                CylindricalSegmentGroup(
                    axis_origin=cf.axis_origin,
                    axis_dir=cf.axis_dir,
                    radius=cf.radius,
                    face_indices=[cf.face_index],
                    u_spans=[cf.u_span],
                    v_spans=[cf.v_span],
                )
            )

    return groups


def group_full_cylinders_to_features(
    full_faces: List[CylindricalFaceInfo],
    axis_ang_tol: float = 1e-3,
    axis_pos_tol: float = 1e-2,
    radius_tol: float = 1e-3,
) -> List[CylindricalFeature]:
    features: List[CylindricalFeature] = []

    for cf in full_faces:
        assigned = False
        for feat in features:
            if not almost_equal(feat.radius, cf.radius, tol=radius_tol):
                continue
            if not vec_almost_equal(feat.axis_dir, cf.axis_dir, ang_tol=axis_ang_tol):
                continue
            dist = dist_between_axes(feat.axis_origin, feat.axis_dir, cf.axis_origin)
            if dist > axis_pos_tol:
                continue

            feat.face_indices.append(cf.face_index)
            feat.faces.append(cf)
            assigned = True
            break

        if not assigned:
            features.append(
                CylindricalFeature(
                    axis_origin=cf.axis_origin,
                    axis_dir=cf.axis_dir,
                    radius=cf.radius,
                    face_indices=[cf.face_index],
                    faces=[cf],
                )
            )

    return features


def split_cyl_features_into_holes_and_bosses(features: List[CylindricalFeature]):
    holes: List[CylindricalFeature] = []
    bosses: List[CylindricalFeature] = []
    ambiguous: List[CylindricalFeature] = []

    for feat in features:
        n_fwd = sum(1 for f in feat.faces if f.orientation == "FORWARD")
        n_rev = sum(1 for f in feat.faces if f.orientation == "REVERSED")

        if n_rev > n_fwd:
            holes.append(feat)
        elif n_fwd > n_rev:
            bosses.append(feat)
        else:
            ambiguous.append(feat)

    return holes, bosses, ambiguous


__all__ = [
    "CylindricalFeature",
    "CylindricalSegmentGroup",
    "group_cylindrical_segments",
    "group_full_cylinders_to_features",
    "split_cyl_features_into_holes_and_bosses",
]
