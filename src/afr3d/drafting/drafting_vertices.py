# afr3d/drafting/dfrafting_vertices.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

from OCC.Core.gp import gp_Pnt
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE
#from OCC.Core import TopExp
from OCC.Core.TopExp import TopExp_Explorer, topexp as TopExp
from OCC.Core.TopoDS import topods
from OCC.Core.TopTools import TopTools_IndexedMapOfShape
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line

from afr3d.drafting.model import (
    DraftView2D,
    DraftVertex2D,
    DraftEdge2D,
)
from afr3d.drafting.analytic import project_point_to_view


# ---------- утилиты ----------

def _vertex_key_from_pnt(p: gp_Pnt, tol: float) -> Tuple[float, float, float]:
    """Квантование координат вершины для дедупликации."""
    return (
        round(p.X() / tol) * tol,
        round(p.Y() / tol) * tol,
        round(p.Z() / tol) * tol,
    )


def _build_vertex_index_map(shape, tol: float) -> Tuple[
    TopTools_IndexedMapOfShape,
    List[gp_Pnt],
    Dict[Tuple[float, float, float], int],
]:
    """
    Строим индексную карту вершин shape:

    - vmap: TopTools_IndexedMapOfShape (TopoDS_Vertex -> index [1..N])
    - points: список gp_Pnt в порядке индексов (0..N-1)
    - key_to_index: карта квантованных координат -> index (0..N-1)
    """
    vmap = TopTools_IndexedMapOfShape()
    TopExp.MapShapes(shape, TopAbs_VERTEX, vmap)

    points: List[gp_Pnt] = []
    key_to_index: Dict[Tuple[float, float, float], int] = {}

    n = vmap.Size()   # количество элементов карты

    for i in range(1, n + 1):
        v = topods.Vertex(vmap.FindKey(i))
        p = BRep_Tool.Pnt(v)
        key = _vertex_key_from_pnt(p, tol)
        idx0 = i - 1
        points.append(p)
        key_to_index[key] = idx0

    return vmap, points, key_to_index


# ---------- публичные функции ----------

def add_vertices_from_topology(
    view: DraftView2D,
    shape,
    dedup_tol: float = 1e-5,
) -> DraftView2D:
    """
    Проецирует ВСЕ вершины shape в координаты вида и добавляет их в view.vertices.

    - DraftVertex2D.id – сквозной id внутри view (продолжаем после имеющихся).
    - DraftVertex2D.source_vertex_index – индекс 3D-вершины (0..N-1)
      в порядковой нумерации TopTools_IndexedMapOfShape.
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=list(view.edges),
        curves=list(view.curves),
    )

    vmap, points, _ = _build_vertex_index_map(shape, dedup_tol)

    next_vid = 0
    if new_view.vertices:
        next_vid = max(v.id for v in new_view.vertices) + 1

    for src_idx, p3d in enumerate(points):
        x2, y2 = project_point_to_view(p3d, new_view.ax2)

        v2d = DraftVertex2D(
            id=next_vid,
            x=float(x2),
            y=float(y2),
            source_vertex_index=src_idx,
        )
        new_view.vertices.append(v2d)
        next_vid += 1

    return new_view


def add_vertices_and_line_edges_from_topology(
    view: DraftView2D,
    shape,
    dedup_tol: float = 1e-5,
    layer: str = "edges_topology",
) -> DraftView2D:
    """
    ДЕБАГ-ФУНКЦИЯ.

    1) Строит карту вершин shape → индекс.
    2) Добавляет в вид проекции всех вершин (DraftVertex2D с source_vertex_index).
    3) Для КАЖДОГО линейного ребра (GeomAbs_Line) находит его концевые вершины
       и добавляет DraftEdge2D, соединяющие соответствующие DraftVertex2D.

    Результат: "телеграфная" проекция топологии:
      - все вершины отмечены звёздочками,
      - между ними проведены прямые сегменты ровно там, где в 3D есть линейные рёбра.
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=list(view.edges),
        curves=list(view.curves),
    )

    # --- 1. карта вершин ---
    vmap, points, key_to_index = _build_vertex_index_map(shape, dedup_tol)

    # --- 2. добавляем 2D-вершины (если ещё не добавляли) ---
    existing_ids = {v.id for v in new_view.vertices}
    next_vid = max(existing_ids) + 1 if existing_ids else 0

    # карта: (source_vertex_index) -> id вершины во view
    srcidx_to_vid: Dict[int, int] = {}

    # быстрый поиск по уже имеющимся вершинам (если ты их добавлял ранее)
    # по source_vertex_index
    for v in new_view.vertices:
        if v.source_vertex_index is not None:
            srcidx_to_vid[v.source_vertex_index] = v.id

    for src_idx, p3d in enumerate(points):
        if src_idx in srcidx_to_vid:
            continue  # уже есть такая вершина в виде

        x2, y2 = project_point_to_view(p3d, new_view.ax2)
        v2d = DraftVertex2D(
            id=next_vid,
            x=float(x2),
            y=float(y2),
            source_vertex_index=src_idx,
        )
        new_view.vertices.append(v2d)
        srcidx_to_vid[src_idx] = next_vid
        next_vid += 1

    # --- 3. добавляем линейные рёбра ---
    next_eid = 0
    if new_view.edges:
        # если у DraftEdge2D есть поле id
        eid_candidates = [getattr(e, "id", None) for e in new_view.edges]
        eid_candidates = [e for e in eid_candidates if e is not None]
        if eid_candidates:
            next_eid = max(eid_candidates) + 1

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp_e.More():
        edge = topods.Edge(exp_e.Current())

        try:
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() != GeomAbs_Line:
                exp_e.Next()
                continue  # сейчас берём только прямые рёбра

            # концевые вершины ребра
            v1 = topods.Vertex(TopExp.FirstVertex(edge))
            v2 = topods.Vertex(TopExp.LastVertex(edge))

            idx1 = vmap.FindIndex(v1) - 1  # → 0-based
            idx2 = vmap.FindIndex(v2) - 1

            vid1 = srcidx_to_vid.get(idx1)
            vid2 = srcidx_to_vid.get(idx2)
            if vid1 is None or vid2 is None:
                exp_e.Next()
                continue  # на всякий случай, но не должно случаться

            e2d = DraftEdge2D(
                id=next_eid,
                v_start=vid1,
                v_end=vid2,
                visible=True,
                layer=layer,
                # если у DraftEdge2D есть такие поля — будут заполнены;
                # если нет — просто убери их.
                source_edge_index=next_eid,
            )
            new_view.edges.append(e2d)
            next_eid += 1

        except Exception:
            # дебаг: одно странное ребро не должно ронять весь вид
            pass

        exp_e.Next()

    return new_view
