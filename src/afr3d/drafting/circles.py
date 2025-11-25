# afr3d/drafting/circles.py

import math
from typing import Dict, List, Optional, Tuple

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

        circle = DraftCurve2D(
            kind=DraftCurveKind.CIRCLE,
            visible=True,
            center=(cx, cy),
            radius=float(h.nominal_radius),
            feature_id=f"hole:{h.id}",
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
