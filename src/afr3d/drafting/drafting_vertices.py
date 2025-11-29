# afr3d/drafting/dfrafting_vertices.py

from __future__ import annotations

import math

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Iterable, Set

from OCC.Core.gp import gp_Pnt
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE
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
    ProjectedSegment2D,
    DraftCurveKind,
    CoverageKind
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
                visible=True, # ИСПРАВИТЬ!
                layer=layer,
                source_edge_index=next_eid,
            )
            new_view.edges.append(e2d)
            next_eid += 1

        except Exception:
            pass

        exp_e.Next()

    return new_view

def build_topology_view_with_hlr_segments(
    shape,
    view_name: str,
    ax2,
    hlr_segments: Iterable[ProjectedSegment2D],
    dedup_tol: float = 1e-5,
    match_tol: float = 1e-2,
    layer_visible: str = "outline_topology",
    layer_hidden: str = "hidden_topology",
    check_unclassified: bool = True,
    unclassified_tol: float = 1e-3,
    debug_stats: bool = True,        # ← чтобы легко включать/выключать сводку
    feature_edge_map: Optional[Dict[int, str]] = None,
) -> DraftView2D:
    """
    Строит DraftView2D, где КАЖДОЕ линейное 3D-ребро порезано на участки
    по HLR-сегментам (ProjectedSegment2D), и каждый участок помечен
    visible=True/False.

    Важные моменты:
      - shape       — исходный TopoDS_Shape.
      - view_name   — "+d1"/"-d2"/... (используется для поворота при отрисовке).
      - ax2         — Тот же базис, что использовался в build_linear_view(shape, ax2).
      - hlr_segments — результат build_linear_view(...).

    debug_stats=True:
      - печатает общее количество рёбер до / после разбиения, длины видимых/скрытых и т.п.
    """
    view = DraftView2D(name=view_name, ax2=ax2)

    # --- 0. 3D-вершины и их 2D-проекции ---
    vmap, points3d, _ = _build_vertex_index_map(shape, dedup_tol)

    proj2d: List[Tuple[float, float]] = []
    for p3d in points3d:
        x2, y2 = project_point_to_view(p3d, ax2)
        proj2d.append((float(x2), float(y2)))

    # карта src_vertex_index -> id вершины во view
    srcidx_to_vid: Dict[int, int] = {}
    next_vid = 0

    def _ensure_vertex(src_idx: int) -> int:
        nonlocal next_vid
        if src_idx in srcidx_to_vid:
            return srcidx_to_vid[src_idx]
        x2, y2 = proj2d[src_idx]
        v2d = DraftVertex2D(
            id=next_vid,
            x=x2,
            y=y2,
            source_vertex_index=src_idx,
        )
        view.vertices.append(v2d)
        srcidx_to_vid[src_idx] = next_vid
        next_vid += 1
        return v2d.id

    # дополнительные вершины (точки разбиения ребра, source_vertex_index=None)
    extra_points: Dict[Tuple[int, float], int] = {}  # (edge_idx, t) -> vid

    def _add_intermediate_vertex(edge_idx: int, x: float, y: float, t: float) -> int:
        nonlocal next_vid
        key = (edge_idx, round(t / dedup_tol) * dedup_tol)
        if key in extra_points:
            return extra_points[key]
        v2d = DraftVertex2D(
            id=next_vid,
            x=x,
            y=y,
            source_vertex_index=None,
        )
        view.vertices.append(v2d)
        extra_points[key] = next_vid
        next_vid += 1
        return v2d.id

    # --- 1. Подготовка HLR-сегментов ---
    hlr_list = list(hlr_segments)
    mt2 = match_tol * match_tol

    # --- 2. Статистика по результатам ---
    edges_total = 0
    edges_with_coverage = 0
    edges_no_coverage = 0
    subedges_total = 0
    subedges_visible = 0
    subedges_hidden = 0
    total_visible_len = 0.0
    total_hidden_len = 0.0

    next_eid = 0

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    edge_idx = 0

    while exp_e.More():
        edge = topods.Edge(exp_e.Current())
        exp_e.Next()
        edges_total += 1

        try:
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() != GeomAbs_Line:
                edge_idx += 1
                continue  # только прямые рёбра

            v1 = topods.Vertex(TopExp.FirstVertex(edge))
            v2 = topods.Vertex(TopExp.LastVertex(edge))

            idx1 = vmap.FindIndex(v1) - 1  # 0-based
            idx2 = vmap.FindIndex(v2) - 1
            if idx1 < 0 or idx2 < 0:
                edge_idx += 1
                continue

            x1, y1 = proj2d[idx1]
            x2, y2 = proj2d[idx2]

            dx = x2 - x1
            dy = y2 - y1
            L = math.hypot(dx, dy)
            if L < 1e-9:
                edge_idx += 1
                continue

            L2 = L * L

            # --- 2a. Собираем интервалы покрытия [t0, t1] из HLR ---
            intervals: List[Tuple[float, float, bool]] = []

            # небольшая оптимизация: заранее локальные функции
            def _dist2_to_line(qx: float, qy: float) -> float:
                vx = qx - x1
                vy = qy - y1
                cross = vx * dy - vy * dx
                # d^2 = cross^2 / |d|^2
                return (cross * cross) / L2

            def _param_t(qx: float, qy: float) -> float:
                vx = qx - x1
                vy = qy - y1
                return (vx * dx + vy * dy) / L2

            for seg in hlr_list:
                q1x, q1y = seg.x1, seg.y1
                q2x, q2y = seg.x2, seg.y2

                # 1) оба конца недалеко от прямой ребра
                if _dist2_to_line(q1x, q1y) > mt2 and _dist2_to_line(q2x, q2y) > mt2:
                    continue

                # 2) параметры по направлению ребра
                t1 = _param_t(q1x, q1y)
                t2 = _param_t(q2x, q2y)

                # если сегмент полностью вне диапазона [0,1] — пропускаем
                if max(t1, t2) < -0.05 or min(t1, t2) > 1.05:
                    continue

                t0 = max(0.0, min(t1, t2))
                t1_ = min(1.0, max(t1, t2))
                if t1_ - t0 < 1e-5:
                    continue

                intervals.append((t0, t1_, seg.visible))

            if not intervals:
                edges_no_coverage += 1
                if debug_stats and edges_no_coverage <= 10:
                    print(f"[WARN] Edge #{edge_idx}: no HLR coverage")
                edge_idx += 1
                continue

            edges_with_coverage += 1

            # --- 2b. Режем [0,1] по всем t-границам ---
            cuts = {0.0, 1.0}
            for t0, t1_, vis in intervals:
                cuts.add(t0)
                cuts.add(t1_)
            cuts_list = sorted(cuts)

            uncovered_len = 0.0

            for i in range(len(cuts_list) - 1):
                a = cuts_list[i]
                b = cuts_list[i + 1]
                if b - a < 1e-5:
                    continue

                mid = 0.5 * (a + b)

                in_any = False
                visible_here = False
                for t0, t1_, vis in intervals:
                    if t0 - 1e-9 <= mid <= t1_ + 1e-9:
                        in_any = True
                        if vis:
                            visible_here = True

                seg_len = (b - a) * L
                if not in_any:
                    uncovered_len += seg_len
                    continue  # этот кусок мы не рисуем

                # 2D-координаты концов подотрезка
                xa = x1 + dx * a
                ya = y1 + dy * a
                xb = x1 + dx * b
                yb = y1 + dy * b

                # вершины (концы)
                if abs(a) < 1e-8:
                    vid_a = _ensure_vertex(idx1)
                elif abs(a - 1.0) < 1e-8:
                    vid_a = _ensure_vertex(idx2)
                else:
                    vid_a = _add_intermediate_vertex(edge_idx, xa, ya, a)

                if abs(b) < 1e-8:
                    vid_b = _ensure_vertex(idx1)
                elif abs(b - 1.0) < 1e-8:
                    vid_b = _ensure_vertex(idx2)
                else:
                    vid_b = _add_intermediate_vertex(edge_idx, xb, yb, b)

                layer = layer_visible if visible_here else layer_hidden

                covered_len = max(L - uncovered_len, 0.0)

                if not intervals:
                    # как у тебя сейчас: no HLR coverage
                    coverage_kind = CoverageKind.NONE
                    coverage_gap = L
                else:
                    # есть хоть какое-то покрытие
                    if uncovered_len < unclassified_tol:
                        coverage_kind = CoverageKind.FULL
                    elif covered_len < unclassified_tol:
                        coverage_kind = CoverageKind.NONE
                    else:
                        coverage_kind = CoverageKind.PARTIAL
                    coverage_gap = uncovered_len
                feat_id = None
                if feature_edge_map is not None:
                    feat_id = feature_edge_map.get(edge_idx)

                e2d = DraftEdge2D(
                    id=next_eid,
                    v_start=vid_a,
                    v_end=vid_b,
                    kind=DraftCurveKind.LINE,
                    visible=visible_here,
                    layer=layer,
                    source_edge_index=edge_idx,
                    feature_id=feat_id,
                    coverage=coverage_kind,
                    coverage_gap=coverage_gap,
                )

                view.edges.append(e2d)
                next_eid += 1
                subedges_total += 1
                if visible_here:
                    subedges_visible += 1
                    total_visible_len += seg_len
                else:
                    subedges_hidden += 1
                    total_hidden_len += seg_len

            if check_unclassified and uncovered_len > unclassified_tol:
                if debug_stats:
                    print(
                        f"[WARN] Edge #{edge_idx}: uncovered length "
                        f"{uncovered_len:.6f} (tol={unclassified_tol})"
                    )

        except Exception as ex:
            if debug_stats:
                print(f"[ERROR] edge #{edge_idx}: {ex}")

        edge_idx += 1

    # --- Итоговая сводка ---
    if debug_stats:
        print(
            "[HLR] edges_total={:d}, with_coverage={:d}, no_coverage={:d}".format(
                edges_total, edges_with_coverage, edges_no_coverage
            )
        )
        print(
            "[HLR] subedges_total={:d} (vis={:d}, hid={:d})".format(
                subedges_total, subedges_visible, subedges_hidden
            )
        )
        print(
            "[HLR] lengths: L_vis={:.6f}, L_hid={:.6f}".format(
                total_visible_len, total_hidden_len
            )
        )

    return view
