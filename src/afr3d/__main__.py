"""CLI entry point for quick feature extraction experiments."""

from __future__ import annotations

import argparse

from afr3d.features import analyze_part, extract_features_from_shape
from afr3d.io.step_import import load_step


def main():
    parser = argparse.ArgumentParser(description="AFR3D playground: planes + cylinders + holes")
    parser.add_argument("step_file", help="Path to file (.stp/.step)")
    args = parser.parse_args()

    shape = load_step(args.step_file)

    part_analysis = analyze_part(shape)

    print("Basic analysis:")

    print(f"Planar faces: {len(part_analysis.planar_faces)}")
    print(f"Full cylinders: {len(part_analysis.full_cyl_faces)}")
    print(f"Partial cylinders: {len(part_analysis.partial_cyl_faces)}")
    print(f"Conical faces: {len(part_analysis.conical_faces)}")

    print(f"\nCylindrical features: {len(part_analysis.cyl_features)}") # is need - ?
    print(f"Hole features: {len(part_analysis.hole_features)}")
    print(f"Boss features: {len(part_analysis.boss_features)}")
    print(f"Ambigious features: {len(part_analysis.ambiguous_features)}")

    print(f"Fillet candidates: {len(part_analysis.fillet_candidates)}")
    print(f"Basic fillets: {len(part_analysis.basic_fillets)}")

    print(f"\nHole AFRs: {len(part_analysis.holes)}")

    holes_afr = part_analysis.holes
    for h in holes_afr:
        print(f"Hole #{h.id}: geom={h.geometry_type}, kind={h.kind}, Rnom={h.nominal_radius}")
        print(f"  segments:")
        for s in h.segments:
            print(f"    {s.kind} R={s.radius} L={s.length:.2f}, faces={s.face_indices}")
        print(f"  chamfers: {[ (c.face_index, round(c.length,3), round(c.semi_angle_deg,1)) for c in h.chamfers ]}")
        print(f"  opening_faces={h.opening_faces}, bottom_faces={h.bottom_faces}")


if __name__ == "__main__":
    main()
