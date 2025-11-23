"""CLI entry point for quick feature extraction experiments."""

from __future__ import annotations

import argparse

from afr3d.features import analyze_part, detect_holes, extract_features_from_shape
from afr3d.io.step_import import load_step


def main():
    parser = argparse.ArgumentParser(description="AFR3D playground: planes + cylinders + holes")
    parser.add_argument("step_file", help="Путь к STEP-файлу (.stp/.step)")
    args = parser.parse_args()

    shape = load_step(args.step_file)

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

    holes_afr = detect_holes(shape)
    part_analysis = analyze_part(shape)

    print(f"Planar faces: {len(planar_faces)}")
    print(f"Full cylinders: {len(full_cyl_faces)}")
    print(f"Partial cylinders: {len(partial_cyl_faces)}")
    print(f"Cylindrical features: {len(cyl_features)}")
    print(f"Hole AFRs: {len(holes_afr)}")
    print(f"Fillet candidates: {len(part_analysis.fillet_candidates)}")


if __name__ == "__main__":
    main()
