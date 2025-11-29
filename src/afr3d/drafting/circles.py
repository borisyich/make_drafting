# afr3d/drafting/circles.py

import math
from math import atan2, pi
from typing import Dict, List, Optional, Tuple, Iterable

from OCC.Core.gp import gp_Dir, gp_Pnt
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopoDS import topods
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Circle

from afr3d.drafting.model import (
    DraftView2D,
    DraftCurve2D,
    DraftCurveKind,
    DraftEdge2D,
    ProjectedSegment2D,
    CoverageKind
)
from afr3d.drafting.analytic import project_point_to_view
from afr3d.features import HoleAFR


def hole_axis_as_gp(hole: HoleAFR) -> Optional[Tuple[gp_Pnt, gp_Dir]]:
    if hole.axis_origin is None or hole.axis_dir is None:
        return None
    ox, oy, oz = hole.axis_origin
    dx, dy, dz = hole.axis_dir
    try:
        p = gp_Pnt(float(ox), float(oy), float(oz))
        d = gp_Dir(float(dx), float(dy), float(dz))
    except Exception:
        return None
    return p, d


def rebuild_circles_from_hlr_analytic(
    view: DraftView2D,
    holes: List[HoleAFR],
    angle_tol_deg: float = 5.0,
    hide_hole_edges: bool = True,
) -> DraftView2D:
    """
    ДЕЛАЕТ КРУГИ ТОЛЬКО ДЛЯ ОТВЕРСТИЙ.

    - Ось отверстия почти параллельна направлению взгляда → рисуем круг.
    - Радиус = nominal_radius.
    - При hide_hole_edges=True скрываем (visible=False) все рёбра,
      принадлежащие граням отверстия (side/opening/bottom) — чтобы
      не было диагоналей/полигонов внутри отверстия.
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=[DraftEdge2D(**e.__dict__) for e in view.edges],
        curves=list(view.curves),
    )

    view_dir: gp_Dir = view.ax2.Direction()
    cos_tol = math.cos(math.radians(angle_tol_deg))

    for h in holes:
        axis_data = hole_axis_as_gp(h)
        if axis_data is None or h.nominal_radius is None:
            continue

        axis_pnt, axis_dir = axis_data

        cos_ang = (
            axis_dir.X() * view_dir.X()
            + axis_dir.Y() * view_dir.Y()
            + axis_dir.Z() * view_dir.Z()
        )
        if abs(cos_ang) < cos_tol:
            continue  # не фронтальное отверстие

        cx, cy = project_point_to_view(axis_pnt, view.ax2)
        
        hole_fid = h.feature_id
        if hole_fid is None:
            if getattr(h, "id", None) is not None:
                hole_fid = f"hole:{h.id}"
            elif getattr(h, "name", None):
                hole_fid = f"hole:{h.name}"
            else:
                hole_fid = "hole:unknown"

        circle = DraftCurve2D(
            kind=DraftCurveKind.CIRCLE,
            visible=True,
            center=(cx, cy),
            radius=float(h.nominal_radius),
            feature_id=hole_fid,
            layer="holes",
        )
        new_view.curves.append(circle)

        if hide_hole_edges:
            hole_faces = set(
                (h.side_face_indices or [])
                + (h.opening_faces or [])
                + (h.bottom_faces or [])
            )
            if not hole_faces:
                continue
            for e in new_view.edges:
                if not e.visible:
                    continue
                if not getattr(e, "face_indices", None):
                    continue
                if hole_faces.intersection(e.face_indices):
                    e.visible = False

    return new_view

def rebuild_full_circles_from_topology(
    view: DraftView2D,
    shape,
    angle_tol_deg: float = 5.0,
    span_tol_deg: float = 6.0,
    dedup_tol: float = 1e-5,
) -> DraftView2D:
    """
    Добавляет только ПОЛНЫЕ окружности из топологии shape.

    - Берём только GeomAbs_Circle.
    - Плоскость круга почти перпендикулярна направлению взгляда.
    - Параметрический span кривой близок к 2*pi (+/- span_tol_deg).
      Дуги (галтели, фаски) отбрасываем — они НЕ превращаются в круги.
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=[DraftEdge2D(**e.__dict__) for e in view.edges],
        curves=list(view.curves),
    )

    view_dir: gp_Dir = view.ax2.Direction()
    cos_tol = math.cos(math.radians(angle_tol_deg))

    circles_map: Dict[Tuple[float, float, float], Tuple[float, float, float]] = {}

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp_e.More():
        edge = topods.Edge(exp_e.Current())
        try:
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() != GeomAbs_Circle:
                exp_e.Next()
                continue

            circ = curve.Circle()
            axis = circ.Axis()
            normal = axis.Direction()

            dot = (
                normal.X()*view_dir.X()
                + normal.Y()*view_dir.Y()
                + normal.Z()*view_dir.Z()
            )
            if abs(dot) < cos_tol:
                exp_e.Next()
                continue  # плоскость круга не фронтальна

            first = curve.FirstParameter()
            last = curve.LastParameter()
            if not (math.isfinite(first) and math.isfinite(last)):
                exp_e.Next()
                continue

            span = abs(last - first)
            full = 2.0 * math.pi
            # допускаем отклонение, но отбрасываем дуги
            if abs(span - full) > math.radians(span_tol_deg):
                exp_e.Next()
                continue  # это дуга (галтель и т.п.), НЕ круг

            center3d = circ.Location()
            cx, cy = project_point_to_view(center3d, view.ax2)
            R = circ.Radius()

            kx = round(cx / dedup_tol) * dedup_tol
            ky = round(cy / dedup_tol) * dedup_tol
            kR = round(R / dedup_tol) * dedup_tol
            key = (kx, ky, kR)

            if key not in circles_map:
                circles_map[key] = (cx, cy, R)

        except Exception:
            pass

        exp_e.Next()

    for cx, cy, R in circles_map.values():
        new_view.curves.append(
            DraftCurve2D(
                kind=DraftCurveKind.CIRCLE,
                visible=True,
                center=(cx, cy),
                radius=R,
                layer="circles_full_topology",
            )
        )

    return new_view

def rebuild_all_circles(
    view: DraftView2D,
    shape,
    holes: List[HoleAFR],
    angle_tol_deg: float = 5.0,
    span_tol_deg: float = 6.0,
    dedup_tol: float = 1e-5,
    hide_hole_edges: bool = True,
) -> DraftView2D:
    """
    1) Полные окружности из топологии (НЕ дуги).
    2) Круги отверстий из HoleAFR, с чисткой рёбер внутри отверстий.
    """
    v1 = rebuild_full_circles_from_topology(
        view=view,
        shape=shape,
        angle_tol_deg=angle_tol_deg,
        span_tol_deg=span_tol_deg,
        dedup_tol=dedup_tol,
    )
    v2 = rebuild_circles_from_hlr_analytic(
        view=v1,
        holes=holes,
        angle_tol_deg=angle_tol_deg,
        hide_hole_edges=hide_hole_edges,
    )
    return v2

def add_arc_curves_from_topology(
    view: DraftView2D,
    shape,
    angle_tol_deg: float = 5.0,
    min_span_deg: float = 5.0,
    max_span_deg: float = 355.0,
) -> DraftView2D:
    """
    Находит ЧАСТИ окружностей (дуги) в рёбрах shape и добавляет их
    как DraftCurve2D(kind=ARC) в координаты вида.

    Важно: здесь мы ИМЕННО дуги, НЕ полные круги.
    - Берём только GeomAbs_Circle.
    - Плоскость окружности почти перпендикулярна направлению взгляда.
    - Параметрический span кривой в градусах: между min_span_deg и max_span_deg.
      То есть исключаем очень маленькие куски и почти полный круг.

    Это чисто ДЕБАГ-ФУНКЦИЯ: чтобы увидеть, где на виде живут скругления, галтели
    и другие "части окружностей".
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=[DraftEdge2D(**e.__dict__) for e in view.edges],
        curves=list(view.curves),
    )

    view_dir: gp_Dir = view.ax2.Direction()
    cos_tol = math.cos(math.radians(angle_tol_deg))

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp_e.More():
        edge = topods.Edge(exp_e.Current())
        try:
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() != GeomAbs_Circle:
                exp_e.Next()
                continue

            circ = curve.Circle()
            axis = circ.Axis()
            normal = axis.Direction()

            dot = (
                normal.X() * view_dir.X()
                + normal.Y() * view_dir.Y()
                + normal.Z() * view_dir.Z()
            )
            # нормаль круга почти параллельна направлению взгляда
            if abs(dot) < cos_tol:
                exp_e.Next()
                continue

            first = curve.FirstParameter()
            last = curve.LastParameter()
            if not (math.isfinite(first) and math.isfinite(last)):
                exp_e.Next()
                continue

            span = abs(last - first)
            span_deg = math.degrees(span)

            # отбрасываем почти нулевые и почти полные
            if span_deg < min_span_deg or span_deg > max_span_deg:
                exp_e.Next()
                continue

            # Берём центр и радиус окружности
            center3d = circ.Location()
            cx, cy = project_point_to_view(center3d, view.ax2)
            R = circ.Radius()

            # Концевые точки дуги
            p_start = curve.Value(first)
            p_end = curve.Value(last)

            sx, sy = project_point_to_view(p_start, view.ax2)
            ex, ey = project_point_to_view(p_end, view.ax2)

            # Углы в 2D (относительно центра), в радианах
            theta_start = math.atan2(sy - cy, sx - cx)
            theta_end = math.atan2(ey - cy, ex - cx)

            arc = DraftCurve2D(
                kind=DraftCurveKind.ARC,
                visible=True,
                center=(cx, cy),
                radius=R,
                start_angle=theta_start,
                end_angle=theta_end,
                layer="arcs_partial",
            )

            # Для удобства можно ещё положить точки начала/конца
            arc.points = [(sx, sy), (ex, ey)]

            new_view.curves.append(arc)

        except Exception:
            # дебаг-режим: не падаем из-за одного ребра
            pass

        exp_e.Next()

    return new_view

def add_arc_polylines_from_topology(
    view: DraftView2D,
    shape,
    angle_tol_deg: float = 5.0,
    min_span_deg: float = 5.0,
    max_span_deg: float = 355.0,
    samples_per_arc: int = 12,
) -> DraftView2D:
    """
    ДЕБАГ-ФУНКЦИЯ.

    Ищет части окружностей (дуги) в рёбрах shape и добавляет их
    как DraftCurve2D(kind=POLYLINE, layer='arcs_partial') в координаты вида.

    - Берём только GeomAbs_Circle.
    - Плоскость круга почти перпендикулярна направлению взгляда.
    - Параметрический span кривой в градусах: между min_span_deg и max_span_deg.
      => почти нулевые куски и почти полный круг (≈360°) отбрасываем.

    Дуги рисуются полилиниями, чтобы избежать ошибок с theta1/theta2.
    """
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=[DraftEdge2D(**e.__dict__) for e in view.edges],
        curves=list(view.curves),
    )

    view_dir: gp_Dir = view.ax2.Direction()
    cos_tol = math.cos(math.radians(angle_tol_deg))

    exp_e = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp_e.More():
        edge = topods.Edge(exp_e.Current())
        try:
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() != GeomAbs_Circle:
                exp_e.Next()
                continue

            circ = curve.Circle()
            axis = circ.Axis()
            normal = axis.Direction()

            dot = (
                normal.X() * view_dir.X()
                + normal.Y() * view_dir.Y()
                + normal.Z() * view_dir.Z()
            )
            # плоскость круга почти фронтальна
            if abs(dot) < cos_tol:
                exp_e.Next()
                continue

            first = curve.FirstParameter()
            last = curve.LastParameter()
            if not (math.isfinite(first) and math.isfinite(last)):
                exp_e.Next()
                continue

            span = abs(last - first)
            span_deg = math.degrees(span)

            # отбрасываем почти нулевые и почти полные окружности
            if span_deg < min_span_deg or span_deg > max_span_deg:
                exp_e.Next()
                continue  # это либо слишком мелкий кусок, либо почти полный круг

            # дискретизация дуги по параметру
            n = max(3, samples_per_arc)
            dt = (last - first) / n
            pts2d: List[Tuple[float, float]] = []
            for i in range(n + 1):
                u = first + i * dt
                p3d = curve.Value(u)
                x2, y2 = project_point_to_view(p3d, view.ax2)
                pts2d.append((float(x2), float(y2)))

            # создаём полилинию
            arc_poly = DraftCurve2D(
                kind=DraftCurveKind.POLYLINE,
                visible=True,
                points=pts2d,
                layer="arcs_partial",
            )
            new_view.curves.append(arc_poly)

        except Exception:
            # дебаг: не роняем всё из-за одного ребра
            pass

        exp_e.Next()

    return new_view


def apply_hlr_visibility_to_circles(
    view: DraftView2D,
    hlr_segments: List[ProjectedSegment2D],
    match_tol: float = 1e-2,          # допуск по радиусу
    angle_eps: float = 1e-4,          # допуск по углу в радианах
    full_circle_tol_deg: float = 3.0, # если покрытие ~360°, можно оставить CIRCLE
    debug: bool = False,
) -> DraftView2D:
    """
    Разбивает окружности из view.curves на видимые/невидимые дуги по маске HLR.

    - Если по окружности вообще нет HLR-сегментов → полностью оставляем как есть.
    - Если сегменты есть → строим покрытие по углу (0..2π) и выдаём набор ARC.
    - Остальные кривые (не CIRCLE) копируем без изменений.

    coverage / coverage_gap:
      - для исходной окружности считаем full_len = 2πR и uncovered_len (где HLR нет);
      - coverage = FULL / PARTIAL / NONE, coverage_gap = uncovered_len.
      - у созданных дуг coverage=FULL, coverage_gap=0.0 (дуга сама полностью определена).
    """
    two_pi = 2.0 * math.pi
    new_view = DraftView2D(
        name=view.name,
        ax2=view.ax2,
        vertices=list(view.vertices),
        edges=[DraftEdge2D(**e.__dict__) for e in view.edges],
        curves=[],
    )

    segs = list(hlr_segments)

    def _norm_angle(a: float) -> float:
        while a < 0.0:
            a += two_pi
        while a >= two_pi:
            a -= two_pi
        return a

    def _add_interval(intervals, a1, a2, visible: bool):
        """Добавляем угловой интервал с учётом перехода через 2π."""
        a1 = _norm_angle(a1)
        a2 = _norm_angle(a2)
        if abs(a1 - a2) < angle_eps:
            return
        if a2 < a1:
            intervals.append((a1, two_pi, visible))
            intervals.append((0.0, a2, visible))
        else:
            intervals.append((a1, a2, visible))

    for c in view.curves:
        if getattr(c, "kind", None) is not DraftCurveKind.CIRCLE:
            # все не-окружности копируем как есть
            new_view.curves.append(c)
            continue

        if c.center is None or c.radius is None:
            new_view.curves.append(c)
            continue

        cx, cy = c.center
        R = float(c.radius)

        # --- 1. собираем интервалы по HLR-сегментам ---
        intervals: List[Tuple[float, float, bool]] = []

        for s in segs:
            mx = 0.5 * (s.x1 + s.x2)
            my = 0.5 * (s.y1 + s.y2)

            dx = mx - cx
            dy = my - cy
            dist = math.hypot(dx, dy)

            if abs(dist - R) > match_tol:
                continue

            a1 = math.atan2(s.y1 - cy, s.x1 - cx)
            a2 = math.atan2(s.y2 - cy, s.x2 - cx)
            _add_interval(intervals, a1, a2, s.visible)

        if not intervals:
            # HLR вообще не "зацепил" окружность — оставляем как есть
            if debug:
                print(
                    f"[CIRC HLR] circle at ({cx:.3f},{cy:.3f}), R={R:.3f}: no coverage"
                )
            # покрытие NONE, вся длина — дырка
            full_len = 2.0 * math.pi * R
            new_circle = DraftCurve2D(
                kind=DraftCurveKind.CIRCLE,
                visible=c.visible,
                center=(cx, cy),
                radius=R,
                edge_ids=list(c.edge_ids),
                feature_id=c.feature_id,
                layer=c.layer or "circles_hlr",
                coverage=CoverageKind.NONE,
                coverage_gap=full_len,
            )
            new_view.curves.append(new_circle)
            continue

        # --- 2. режем [0,2π] по границам интервалов ---
        cuts = {0.0, two_pi}
        for a0, a1, vis in intervals:
            cuts.add(_norm_angle(a0))
            cuts.add(_norm_angle(a1))

        cuts_list = sorted(cuts)
        merged_cuts = [cuts_list[0]]
        for a in cuts_list[1:]:
            if abs(a - merged_cuts[-1]) > angle_eps:
                merged_cuts.append(a)
        cuts_list = merged_cuts

        arc_parts: List[Tuple[float, float, bool]] = []
        uncovered_len = 0.0

        for i in range(len(cuts_list) - 1):
            a0 = cuts_list[i]
            a1 = cuts_list[i + 1]
            if a1 - a0 < angle_eps:
                continue

            mid = _norm_angle(0.5 * (a0 + a1))

            in_any = False
            visible_here = False
            for t0, t1, vis in intervals:
                if t0 - angle_eps <= mid <= t1 + angle_eps:
                    in_any = True
                    if vis:
                        visible_here = True

            seg_len = (a1 - a0) * R
            if not in_any:
                uncovered_len += seg_len
                continue

            arc_parts.append((a0, a1, visible_here))

        full_len = 2.0 * math.pi * R
        covered_len = max(full_len - uncovered_len, 0.0)

        if uncovered_len < 1e-3:
            coverage_kind = CoverageKind.FULL
        elif covered_len < 1e-3:
            coverage_kind = CoverageKind.NONE
        else:
            coverage_kind = CoverageKind.PARTIAL

        coverage_gap = uncovered_len

        if debug:
            print(
                f"[CIRC HLR] circle at ({cx:.3f},{cy:.3f}), R={R:.3f}: "
                f"uncovered_len={uncovered_len:.6f}, "
                f"coverage={coverage_kind.name}"
            )

        if not arc_parts:
            # формально HLR что-то дал, но всё ушло в uncovered → как NONE
            new_circle = DraftCurve2D(
                kind=DraftCurveKind.CIRCLE,
                visible=c.visible,
                center=(cx, cy),
                radius=R,
                edge_ids=list(c.edge_ids),
                feature_id=c.feature_id,
                layer=c.layer or "circles_hlr",
                coverage=coverage_kind,
                coverage_gap=coverage_gap,
            )
            new_view.curves.append(new_circle)
            continue

        # --- 3. если покрытие почти полное и одной видимости → оставляем CIRCLE ---
        total_angle = sum(b - a for a, b, _ in arc_parts)
        all_visible = all(vis for _, _, vis in arc_parts)
        all_hidden = all((not vis) for _, _, vis in arc_parts)

        full_circle_tol = math.radians(full_circle_tol_deg)

        if abs(total_angle - two_pi) < full_circle_tol and (all_visible or all_hidden):
            vis = all_visible
            new_circle = DraftCurve2D(
                kind=DraftCurveKind.CIRCLE,
                visible=vis,
                center=(cx, cy),
                radius=R,
                edge_ids=list(c.edge_ids),
                feature_id=c.feature_id,
                layer=c.layer or "circles_hlr",
                coverage=coverage_kind,
                coverage_gap=coverage_gap,
            )
            new_view.curves.append(new_circle)
            continue

        # --- 4. иначе окружность разбиваем на дуги ARC ---
        for a0, a1, vis in arc_parts:
            arc = DraftCurve2D(
                kind=DraftCurveKind.ARC,
                visible=vis,
                center=(cx, cy),
                radius=R,
                start_angle=a0,
                end_angle=a1,
                edge_ids=list(c.edge_ids),
                feature_id=c.feature_id,
                layer=c.layer or "circles_hlr",
                coverage=CoverageKind.FULL,
                coverage_gap=0.0,
            )
            new_view.curves.append(arc)

    return new_view
