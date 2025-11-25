# afr3d/drafting/analytic.py

import math
from typing import Dict, List
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE
from OCC.Core.TopoDS import topods
from OCC.Core.BRep import BRep_Tool

from afr3d.drafting.model import (
    DraftVertex2D, DraftEdge2D, DraftView2D, DraftCurveKind, Point2D
)

def project_point_to_view(pnt: gp_Pnt, ax2: gp_Ax2) -> Point2D:
    """
    Проекция 3D-точки pnt в 2D координаты (ξ, η) плоскости вида ax2.
    """
    origin = ax2.Location()
    xdir = ax2.XDirection()
    ydir = ax2.YDirection()

    v = gp_Vec(origin, pnt)

    vx = v.Dot(gp_Vec(xdir.X(), xdir.Y(), xdir.Z()))
    vy = v.Dot(gp_Vec(ydir.X(), ydir.Y(), ydir.Z()))
    return float(vx), float(vy)


def build_analytic_view_from_topology(shape, ax2: gp_Ax2, name: str = "analytic") -> DraftView2D:
    """
    Аналитический вид: проекция всех вершин и рёбер BRep на плоскость вида.
    НЕ учитывает видимость, скрытые линии, etc.
    """
    view = DraftView2D(name=name, ax2=ax2)

    # === 0. Собираем все грани и даём им индексы ===
    faces: List["TopoDS_Face"] = []
    exp_f = TopExp_Explorer(shape, TopAbs_FACE)
    while exp_f.More():
        faces.append(topods.Face(exp_f.Current()))
        exp_f.Next()

    # face_idx -> [TopoDS_Edge]
    face_edges: List[List["TopoDS_Edge"]] = [[] for _ in range(len(faces))]
    # Edge TShape key -> set(face_idx)
    edge_faces_map: Dict[int, List[int]] = {}

    for fi, face in enumerate(faces):
        exp_e = TopExp_Explorer(face, TopAbs_EDGE)
        while exp_e.More():
            e = topods.Edge(exp_e.Current())
            key = e.TShape().__hash__()  # грубый, но работает как идентификатор
            face_edges[fi].append(e)
            edge_faces_map.setdefault(key, []).append(fi)
            exp_e.Next()
    
    # === 1. Собираем вершины ===
    vertex_map: Dict[int, DraftVertex2D] = {}  # id -> DraftVertex2D
    vertex_index_map: Dict[str, int] = {}      # ключ (координаты) -> id
    next_vid = 0

    exp_v = TopExp_Explorer(shape, TopAbs_VERTEX)
    while exp_v.More():
        v = topods.Vertex(exp_v.Current())
        p = BRep_Tool.Pnt(v)
        key = f"{p.X():.9f}_{p.Y():.9f}_{p.Z():.9f}"

        if key not in vertex_index_map:
            vid = next_vid
            next_vid += 1
            x2d, y2d = project_point_to_view(p, ax2)
            dv = DraftVertex2D(
                id=vid,
                x=x2d,
                y=y2d,
                source_vertex_index=vid,  # пока просто индекс по порядку
            )
            vertex_map[vid] = dv
            vertex_index_map[key] = vid

        exp_v.Next()

    view.vertices = list(vertex_map.values())

    # === 2. Собираем рёбра ===
    edges: List[DraftEdge2D] = []
    next_eid = 0

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp_e.More():
        e = topods.Edge(exp_e.Current())
        key = e.TShape().__hash__()

        v1_3d = topexp.FirstVertex(e)
        v2_3d = topexp.LastVertex(e)

        p1 = BRep_Tool.Pnt(v1_3d)
        p2 = BRep_Tool.Pnt(v2_3d)

        key1 = f"{p1.X():.9f}_{p1.Y():.9f}_{p1.Z():.9f}"
        key2 = f"{p2.X():.9f}_{p2.Y():.9f}_{p2.Z():.9f}"

        # если почему-то вершины не были в общем списке – добавим
        for p, key in [(p1, key1), (p2, key2)]:
            if key not in vertex_index_map:
                vid = next_vid
                next_vid += 1
                x2d, y2d = project_point_to_view(p, ax2)
                dv = DraftVertex2D(
                    id=vid,
                    x=x2d,
                    y=y2d,
                    source_vertex_index=vid,
                )
                vertex_map[vid] = dv
                vertex_index_map[key] = vid

        v_start = vertex_index_map[key1]
        v_end   = vertex_index_map[key2]

        de = DraftEdge2D(
            id=next_eid,
            v_start=v_start,
            v_end=v_end,
            kind=DraftCurveKind.LINE,  # пока всё как линейные сегменты
            visible=True,              # видимость пока не учитываем
            source_edge_index=next_eid,
            face_indices=edge_faces_map.get(key, []),
        )
        edges.append(de)
        next_eid += 1

        exp_e.Next()

    view.edges = edges
    view.vertices = list(vertex_map.values())

    return view
