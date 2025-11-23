"""Small debug utilities for inspecting face neighborhoods."""

from __future__ import annotations

from afr3d.geom.primitives import CylindricalFaceInfo, PlanarFaceInfo


def debug_face_neighbors(
    face_index: int,
    planar_by_index: dict[int, PlanarFaceInfo],
    cyl_by_index: dict[int, CylindricalFaceInfo],
    adj: dict[int, set[int]],
):
    print(f"\nFace {face_index}:")
    if face_index in planar_by_index:
        print("  Type: PLANAR")
        print("   ", planar_by_index[face_index])
    elif face_index in cyl_by_index:
        print("  Type: CYL")
        print("   ", cyl_by_index[face_index])
    else:
        print("  Type: OTHER")

    for ni in sorted(adj.get(face_index, [])):
        if ni in planar_by_index:
            t = "PLANAR"
        elif ni in cyl_by_index:
            t = "CYL"
        else:
            t = "OTHER"
        print(f"    neighbor {ni}: {t}")


__all__ = ["debug_face_neighbors"]
