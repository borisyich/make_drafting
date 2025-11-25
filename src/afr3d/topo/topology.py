"""Topology helpers for traversing shapes and extracting faces."""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_FORWARD, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_IndexedMapOfShape,
    TopTools_ListIteratorOfListOfShape,
)
from OCC.Core.TopoDS import topods, TopoDS_Shape

def to_face(shape):
    return topods.Face(shape)

from afr3d.geom.primitives import (
    ConicalFaceInfo,
    CylindricalFaceInfo,
    PlanarFaceInfo,
    angle_between_normals_deg,
    gp_dir_to_tuple,
    gp_pnt_to_tuple,
    normalize_vec,
)


def indexed_map_size(indexed) -> int:
    if hasattr(indexed, "Extent"):
        return indexed.Extent()
    if hasattr(indexed, "Size"):
        return indexed.Size()
    try:
        return len(indexed)
    except TypeError as exc:
        raise AttributeError("Indexed map has neither Extent() nor Size() nor __len__") from exc


def collect_faces_and_map(shape: TopoDS_Shape):
    face_map = TopTools_IndexedMapOfShape()
    topexp.MapShapes(shape, TopAbs_FACE, face_map)

    n_faces = indexed_map_size(face_map)
    faces: List[topods.Face] = []
    for i in range(1, n_faces + 1):
        faces.append(to_face(face_map.FindKey(i)))
    return faces, face_map


def build_face_adjacency(shape: TopoDS_Shape, face_map):
    n_faces = indexed_map_size(face_map)
    adj: Dict[int, set[int]] = {i: set() for i in range(n_faces)}

    edge2faces = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge2faces)

    n_edges = indexed_map_size(edge2faces)
    for i in range(1, n_edges + 1):
        faces_list = edge2faces.FindFromIndex(i)
        face_indices: List[int] = []
        it = TopTools_ListIteratorOfListOfShape(faces_list)
        while it.More():
            f = to_face(it.Value())
            idx_1b = face_map.FindIndex(f)
            if idx_1b > 0:
                face_indices.append(idx_1b - 1)
            it.Next()

        for a in range(len(face_indices)):
            for b in range(a + 1, len(face_indices)):
                fa = face_indices[a]
                fb = face_indices[b]
                if fa != fb:
                    adj[fa].add(fb)
                    adj[fb].add(fa)

    return adj


def extract_planar_faces(shape: TopoDS_Shape) -> List[PlanarFaceInfo]:
    result: List[PlanarFaceInfo] = []

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    while exp.More():
        face = to_face(exp.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        surf_type = adaptor.GetType()

        if surf_type == GeomAbs_Plane:
            plane = adaptor.Plane()
            normal = plane.Axis().Direction()
            loc = plane.Location()

            gprops = GProp_GProps()
            brepgprop.SurfaceProperties(face, gprops)
            area = gprops.Mass()

            ori = face.Orientation()
            if ori == TopAbs_FORWARD:
                orientation = "FORWARD"
            elif ori == TopAbs_REVERSED:
                orientation = "REVERSED"
            else:
                orientation = str(ori)

            result.append(
                PlanarFaceInfo(
                    face_index=face_index,
                    point=gp_pnt_to_tuple(loc),
                    normal=normalize_vec(gp_dir_to_tuple(normal)),
                    area=area,
                    orientation=orientation,
                )
            )

        face_index += 1
        exp.Next()

    return result


def extract_cylindrical_faces(
    shape: TopoDS_Shape, full_circle_tol_deg: float = 5.0
) -> Tuple[List[CylindricalFaceInfo], List[CylindricalFaceInfo]]:
    full_cyl_faces: List[CylindricalFaceInfo] = []
    partial_cyl_faces: List[CylindricalFaceInfo] = []
    full_circle_tol = math.radians(full_circle_tol_deg)

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    while exp.More():
        face = to_face(exp.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        surf_type = adaptor.GetType()

        if surf_type == GeomAbs_Cylinder:
            cyl = adaptor.Cylinder()
            axis = cyl.Axis()
            radius = cyl.Radius()

            u_first = adaptor.FirstUParameter()
            u_last = adaptor.LastUParameter()
            v_first = adaptor.FirstVParameter()
            v_last = adaptor.LastVParameter()
            u_span = abs(u_last - u_first)
            v_span = abs(v_last - v_first)

            axis_origin = gp_pnt_to_tuple(axis.Location())
            axis_dir = normalize_vec(gp_dir_to_tuple(axis.Direction()))

            ori = face.Orientation()
            if ori == TopAbs_FORWARD:
                orientation = "FORWARD"
            elif ori == TopAbs_REVERSED:
                orientation = "REVERSED"
            else:
                orientation = str(ori)

            gprops = GProp_GProps()
            brepgprop.SurfaceProperties(face, gprops)
            area = gprops.Mass()

            u_mid = 0.5 * (u_first + u_last)
            v_mid = 0.5 * (v_first + v_last)
            mid_point = gp_pnt_to_tuple(adaptor.Value(u_mid, v_mid))

            info = CylindricalFaceInfo(
                face_index=face_index,
                axis_origin=axis_origin,
                axis_dir=axis_dir,
                radius=radius,
                u_first=u_first,
                u_last=u_last,
                u_span=u_span,
                v_first=v_first,
                v_last=v_last,
                v_span=v_span,
                is_u_closed=adaptor.IsUClosed(),
                is_v_closed=adaptor.IsVClosed(),
                orientation=orientation,
                area=area,
                mid_point=mid_point,
                topods_face=face,
            )

            if abs(u_span - 2 * math.pi) <= full_circle_tol:
                full_cyl_faces.append(info)
            else:
                partial_cyl_faces.append(info)

        face_index += 1
        exp.Next()

    return full_cyl_faces, partial_cyl_faces


def extract_conical_faces(shape: TopoDS_Shape) -> List[ConicalFaceInfo]:
    con_faces: List[ConicalFaceInfo] = []

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    while exp.More():
        face = to_face(exp.Current())
        adaptor = BRepAdaptor_Surface(face, True)

        if adaptor.GetType() == GeomAbs_Cone:
            cone = adaptor.Cone()

            apex = gp_pnt_to_tuple(cone.Apex())
            axis_dir = normalize_vec(gp_dir_to_tuple(cone.Axis().Direction()))
            semi_angle_deg = math.degrees(cone.SemiAngle())
            ref_radius = cone.RefRadius()

            u_first = adaptor.FirstUParameter()
            u_last = adaptor.LastUParameter()
            v_first = adaptor.FirstVParameter()
            v_last = adaptor.LastVParameter()
            u_span = abs(u_last - u_first)
            v_span = abs(v_last - v_first)

            u_mid = 0.5 * (u_first + u_last)
            p1 = adaptor.Value(u_mid, v_first)
            p2 = adaptor.Value(u_mid, v_last)
            length = math.dist(gp_pnt_to_tuple(p1), gp_pnt_to_tuple(p2))

            con_faces.append(
                ConicalFaceInfo(
                    face_index=face_index,
                    apex=apex,
                    axis_dir=axis_dir,
                    semi_angle_deg=semi_angle_deg,
                    ref_radius=ref_radius,
                    u_span=u_span,
                    v_span=v_span,
                    length=length,
                )
            )

        face_index += 1
        exp.Next()

    return con_faces


def get_part_max_dim(shape: TopoDS_Shape) -> float:
    obb = Bnd_OBB()
    brepbndlib.AddOBB(shape, obb, True, True, True)
    m = 2.0 * max(obb.XHSize(), obb.YHSize(), obb.ZHSize())
    return m if m > 1e-6 else 100.0


__all__ = [
    "collect_faces_and_map",
    "build_face_adjacency",
    "extract_planar_faces",
    "extract_cylindrical_faces",
    "extract_conical_faces",
    "get_part_max_dim",
    "indexed_map_size",
]
