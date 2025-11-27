from __future__ import annotations

import matplotlib.pyplot as plt
import math

from dataclasses import dataclass
from typing import List, Tuple

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.gp import gp_Dir, gp_Pnt, gp_Ax2, gp_Vec
from OCC.Core.HLRAlgo import HLRAlgo_Projector
from OCC.Core.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.GProp import GProp_GProps

from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_SOLID
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopoDS import TopoDS_Compound

from OCC.Core.BRep import BRep_Builder

from OCC.Core.TopoDS import topods

def to_edge(shape):
    return topods.Edge(shape)

from OCC.Core.GeomAbs import (
    GeomAbs_Line,
    GeomAbs_Circle,
    GeomAbs_Ellipse,
    GeomAbs_BSplineCurve,
    GeomAbs_BezierCurve,
)

from afr3d.drafting.model import ViewHLRStats, OrthoViewDef, ViewSet, ProjectedSegment2D

# --- Low-level utilities ---

def _safe_edge_length(edge) -> float:
    """
    Аккуратно считаем длину одного ребра.
    Любая проблема внутри OCC → длина 0, чтобы не ронять процесс.
    """
    try:
        curve = BRepAdaptor_Curve(edge)
        first = curve.FirstParameter()
        last = curve.LastParameter()
        if not math.isfinite(first) or not math.isfinite(last) or last <= first:
            return 0.0

        length = GCPnts_AbscissaPoint.Length(curve, first, last)
        if not math.isfinite(length) or length < 0:
            return 0.0
        return float(length)
    except Exception as e:
        print(e)
        # Любая C++/Python ошибка внутри адаптера/длины → просто пропускаем
        return 0.0


def _sum_edge_lengths(shape) -> float:
    """Суммирует длину всех рёбер в shape через перебор EDGE + BRepAdaptor_Curve."""
    if shape is None or shape.IsNull():
        return 0.0

    total = 0.0
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        edge = to_edge(exp.Current())
        total += _safe_edge_length(edge)
        exp.Next()
    return total


def _sum_edge_lengths_from_compounds(shapes) -> float:
    total = 0.0
    for s in shapes:
        if s is None or s.IsNull():
            continue
        total += _sum_edge_lengths(s)
    return total


def _run_hlr_for_direction(shape, view_dir: gp_Dir) -> Tuple[float, float]:
    """
    Запускает HLR для заданного направления взгляда.
    Возвращает (L_vis, L_hid).
    """
    # Ортогональная проекция вдоль view_dir, "камера" в начале координат
    projector = HLRAlgo_Projector(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), view_dir))

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr_shapes = HLRBRep_HLRToShape(algo)

    # Видимые линии
    visibles = [
        hlr_shapes.VCompound(),
        hlr_shapes.OutLineVCompound(),
        hlr_shapes.Rg1LineVCompound(),
        hlr_shapes.RgNLineVCompound(),
    ]

    # Скрытые линии
    hiddens = [
        hlr_shapes.HCompound(),
        hlr_shapes.OutLineHCompound(),
        hlr_shapes.Rg1LineHCompound(),
        hlr_shapes.RgNLineHCompound(),
    ]

    L_vis = _sum_edge_lengths_from_compounds(visibles)
    L_hid = _sum_edge_lengths_from_compounds(hiddens)
    return L_vis, L_hid


def _compute_obb(shape):
    """
    Строит OBB для shape.
    Возвращает:
        center: gp_Pnt
        axes: (d1, d2, d3) как gp_Dir
        sizes: (sx, sy, sz) — полные размеры
    """
    obb = Bnd_OBB()
    brepbndlib.AddOBB(shape, obb, True, True, True)

    c = obb.Center()  # в некоторых версиях это не чистый gp_Pnt
    center = gp_Pnt(c.X(), c.Y(), c.Z())

    d1 = obb.XDirection()
    d2 = obb.YDirection()
    d3 = obb.ZDirection()

    sx = 2.0 * obb.XHSize()
    sy = 2.0 * obb.YHSize()
    sz = 2.0 * obb.ZHSize()

    return center, (d1, d2, d3), (sx, sy, sz)


def _gpdir_scaled(dir_: gp_Dir, sign: int) -> gp_Dir:
    """Возвращает gp_Dir с тем же направлением, но с нужным знаком."""
    return gp_Dir(sign * dir_.X(), sign * dir_.Y(), sign * dir_.Z())


def _normalize_vec(x: float, y: float, z: float) -> gp_Dir:
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("Zero-length vector in _normalize_vec")
    return gp_Dir(x / norm, y / norm, z / norm)


# --- Step 2: выбор front/top/side по OBB + HLR ---

def analyze_view_candidates(shape) -> List[ViewHLRStats]:
    """
    Строит OBB и считает HLR-метрики для шести направлений ±d1, ±d2, ±d3.
    Возвращает список ViewHLRStats.
    """
    _, (d1, d2, d3), (sx, sy, sz) = _compute_obb(shape)

    # A_obb по формуле из твоего плана:
    # A_obb(±d1) = sy * sz,
    # A_obb(±d2) = sx * sz,
    # A_obb(±d3) = sx * sy.

    areas = [
        sy * sz,  # d1
        sx * sz,  # d2
        sx * sy,  # d3
    ]
    dirs = [d1, d2, d3]

    candidates: List[ViewHLRStats] = []

    for axis_index, (axis_dir, A_obb) in enumerate(zip(dirs, areas)):
        for sign in (+1, -1):
            view_dir = _gpdir_scaled(axis_dir, sign)
            L_vis, L_hid = _run_hlr_for_direction(shape, view_dir)
            candidates.append(
                ViewHLRStats(
                    axis_index=axis_index,
                    sign=sign,
                    direction=view_dir,
                    A_obb=A_obb,
                    L_vis=L_vis,
                    L_hid=L_hid,
                )
            )

    return candidates


def run_hlr_and_get_ratio_hidden(shape, view_dir: gp_Dir) -> float:
    """
    Быстрый HLR-тестирование: считаем L_hidden/(L_visible+L_hidden).
    """
    origin = gp_Pnt(0,0,0)
    projector = HLRAlgo_Projector(gp_Ax2(origin, view_dir))

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr = HLRBRep_HLRToShape(algo)

    visible_comps = [
        hlr.VCompound(),
        hlr.OutLineVCompound(),
        hlr.Rg1LineVCompound(),
        hlr.RgNLineVCompound(),
    ]

    hidden_comps = [
        hlr.HCompound(),
        hlr.OutLineHCompound(),
        hlr.Rg1LineHCompound(),
        hlr.RgNLineHCompound(),
    ]

    def sum_len(comps):
        total = 0.0
        for comp in comps:
            if comp is None or comp.IsNull():
                continue
            exp = TopExp_Explorer(comp, TopAbs_EDGE)
            while exp.More():
                edge = to_edge(exp.Current())
                try:
                    c = BRepAdaptor_Curve(edge)
                    f, l = c.FirstParameter(), c.LastParameter()
                    if l > f:
                        p1 = c.Value(f)
                        p2 = c.Value(l)
                        dx = p2.X() - p1.X()
                        dy = p2.Y() - p1.Y()
                        dz = p2.Z() - p1.Z()
                        total += math.sqrt(dx*dx + dy*dy + dz*dz)
                except Exception:
                    pass
                exp.Next()
        return total

    L_vis = sum_len(visible_comps)
    L_hid = sum_len(hidden_comps)
    if L_vis + L_hid < 1e-12:
        return 1.0
    return L_hid / (L_vis + L_hid)


def select_main_views(shape) -> ViewSet:
    """
    Выбор front/top/side:
    - кандидаты ±d1, ±d2, ±d3 уже посчитаны в analyze_view_candidates(shape);
    - front: max A_obb, при близких A_obb — min R_hidden;
    - top/side: оставшиеся оси, выбор top по A_obb+R_hidden;
    - базис объекта строим из выбранных front и top;
    - для каждого вида строим gp_Ax2(origin, Zdir, Xdir).
    """
    # 1) OBB: центр + оси нам пригодятся для origin
    center, _, _ = _compute_obb(shape)
    origin = gp_Pnt(center.X(), center.Y(), center.Z())

    # 2) Кандидаты на виды (+/- оси OBB с HLR-метриками)
    candidates = analyze_view_candidates(shape)  # уже есть у тебя

    # 3) FRONT: сортируем по (-A_obb, R_hidden)
    sorted_for_front = sorted(
        candidates,
        key=lambda c: (-c.A_obb, c.R_hidden)
    )
    front_cand = sorted_for_front[0]

    # 4) Оставшиеся оси -> претенденты на top/side
    remaining_axis_indices = {0, 1, 2} - {front_cand.axis_index}

    axis_best = {}
    for idx in remaining_axis_indices:
        cand_for_axis = [c for c in candidates if c.axis_index == idx]
        # для каждой оси выбираем знак с минимальным R_hidden
        best = min(cand_for_axis, key=lambda c: c.R_hidden)
        axis_best[idx] = best

    # 5) Выбираем top из двух оставшихся осей (A_obb, R_hidden)
    axis_items = list(axis_best.items())
    (_, cand1), (_, cand2) = axis_items

    def _score_for_top(c):
        return (-c.A_obb, c.R_hidden)

    top_cand = cand1
    side_cand = cand2
    if _score_for_top(side_cand) < _score_for_top(top_cand):
        top_cand, side_cand = side_cand, top_cand

    # 6) Строим ортонормированный базис объекта:
    #    z_obj = -front_dir, y_obj = -top_dir, x_obj = y_obj × z_obj
    front_dir = front_cand.direction  # gp_Dir
    top_dir   = top_cand.direction    # gp_Dir

    z_obj = _normalize_vec(-front_dir.X(), -front_dir.Y(), -front_dir.Z())
    y_obj = _normalize_vec(-top_dir.X(),   -top_dir.Y(),   -top_dir.Z())

    cx = y_obj.Y() * z_obj.Z() - y_obj.Z() * z_obj.Y()
    cy = y_obj.Z() * z_obj.X() - y_obj.X() * z_obj.Z()
    cz = y_obj.X() * z_obj.Y() - y_obj.Y() * z_obj.X()
    x_obj = _normalize_vec(cx, cy, cz)

    # 7) Для каждого вида строим Ax2:
    #    - front: Z_view = -z_obj (взгляд на деталь), X_view = x_obj
    #    - top:   Z_view = -y_obj, X_view = x_obj
    #    - side:  Z_view = -x_obj, X_view = z_obj (правый вид справа)
    front_Z = _normalize_vec(-z_obj.X(), -z_obj.Y(), -z_obj.Z())
    front_X = x_obj
    ax2_front = gp_Ax2(origin, front_Z, front_X)

    top_Z = _normalize_vec(-y_obj.X(), -y_obj.Y(), -y_obj.Z())
    top_X = x_obj
    ax2_top = gp_Ax2(origin, top_Z, top_X)

    side_Z = _normalize_vec(-x_obj.X(), -x_obj.Y(), -x_obj.Z())
    side_X = z_obj
    ax2_side = gp_Ax2(origin, side_Z, side_X)

    return ViewSet(
        front=OrthoViewDef("front", ax2_front),
        top=OrthoViewDef("top",   ax2_top),
        side=OrthoViewDef("side", ax2_side),
    )


def generate_six_obb_views(shape) -> List[OrthoViewDef]:
    center, (d1, d2, d3), _ = _compute_obb(shape)
    origin = gp_Pnt(center.X(), center.Y(), center.Z())

    views: List[OrthoViewDef] = []

    def make_ax2(name: str, dir_main: gp_Dir, dir_x_hint: gp_Dir) -> OrthoViewDef:
        # строим ось: Z = dir_main, X = ортогональ к Z в плоскости (dir_x_hint, Z)
        z = gp_Dir(dir_main.X(), dir_main.Y(), dir_main.Z())
        # берём вектор, не коллинеарный z, и орто-нормируем
        tmp = gp_Dir(dir_x_hint.X(), dir_x_hint.Y(), dir_x_hint.Z())
        vx = gp_Vec(tmp.X(), tmp.Y(), tmp.Z())
        vz = gp_Vec(z.X(), z.Y(), z.Z())
        vx.Cross(vz)
        if vx.Magnitude() < 1e-9:
            vx = gp_Vec(1.0, 0.0, 0.0)
        vx.Normalize()
        xdir = gp_Dir(vx.X(), vx.Y(), vx.Z())
        ax2 = gp_Ax2(origin, z, xdir)
        return OrthoViewDef(name=name, ax2=ax2)

    # d1, d2, d3 — ортонормальны
    views.append(make_ax2("+d1", d1, d2))
    views.append(make_ax2("-d1", gp_Dir(-d1.X(), -d1.Y(), -d1.Z()), d2))
    views.append(make_ax2("+d2", d2, d3))
    views.append(make_ax2("-d2", gp_Dir(-d2.X(), -d2.Y(), -d2.Z()), d3))
    views.append(make_ax2("+d3", d3, d1))
    views.append(make_ax2("-d3", gp_Dir(-d3.X(), -d3.Y(), -d3.Z()), d1))

    return views

# def _project_point_to_uv(p: gp_Pnt, origin: gp_Pnt, u: gp_Dir, v: gp_Dir) -> tuple[float, float]:
#     """
#     Проекция 3D-точки p в 2D-координаты (ξ, η) в базисе (u, v) с центром origin.
#     Вариант БЕЗ gp_Vec, чтобы не ловить problemer с перегрузками в 7.9.0.
#     """
#     # вектор от origin к p как просто разность координат
#     vx = p.X() - origin.X()
#     vy = p.Y() - origin.Y()
#     vz = p.Z() - origin.Z()

#     # скалярное произведение с направлениями u и v
#     xi  = vx * u.X() + vy * u.Y() + vz * u.Z()
#     eta = vx * v.X() + vy * v.Y() + vz * v.Z()

#     return float(xi), float(eta)


def extract_solids(root_shape):
    """
    Возвращает compound, содержащий только SOLID'ы из исходного shape.
    Это отсекает текст, разметку, вспомогательные тела и т.п.
    """
    if root_shape is None or root_shape.IsNull():
        return root_shape

    exp = TopExp_Explorer(root_shape, TopAbs_SOLID)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)

    has_solids = False
    while exp.More():
        solid = topods.Solid(exp.Current())
        builder.Add(comp, solid)
        has_solids = True
        exp.Next()

    return comp if has_solids else root_shape

def _edge_to_poly_segments_2d(edge,
                              min_seg: int = 1,
                              max_seg: int = 16,
                              default_seg: int = 6):
    """
    Преобразует одно HLR-ребро в набор 2D-отрезков.
    - Линия → 1 сегмент.
    - Круг/эллипс → больше сегментов, но не более max_seg.
    - Сплайны/Безье → default_seg.
    """
    segs = []

    curve = BRepAdaptor_Curve(edge)
    ctype = curve.GetType()
    first = curve.FirstParameter()
    last = curve.LastParameter()

    if not math.isfinite(first) or not math.isfinite(last) or last <= first:
        return segs

    # длина по параметру (примерно примем её как меру "протяжённости")
    approx_len = last - first

    if ctype == GeomAbs_Line:
        n = 1
    elif ctype in (GeomAbs_Circle, GeomAbs_Ellipse):
        # чем больше дуга, тем больше сегментов, но не бешено
        # полный круг: n ~ max_seg, маленькая дуга: меньше
        # просто эвристика:
        n = int(max(min_seg, min(max_seg, default_seg * approx_len / (2 * math.pi))))
        n = max(n, 6)  # для круга всё же минимум 6 сегментов
    elif ctype in (GeomAbs_BSplineCurve, GeomAbs_BezierCurve):
        n = default_seg
    else:
        n = min_seg

    n = max(min_seg, min(n, max_seg))

    # дискретизация по параметру
    params = [first + (last-first)*i/n for i in range(n+1)]
    pts = [curve.Value(u) for u in params]

    for i in range(n):
        p1, p2 = pts[i], pts[i+1]
        x1, y1 = float(p1.X()), float(p1.Y())
        x2, y2 = float(p2.X()), float(p2.Y())
        if (x1-x2)**2 + (y1-y2)**2 < 1e-12:
            continue
        segs.append((x1, y1, x2, y2))

    return segs


def build_linear_view_debug(shape, view_ax2) -> List[ProjectedSegment2D]:
    """
    Debug-режим: строим вид как VCompound + OutLineVCompound,
    с дискретизацией дуг/сплайнов. Без Rg* и скрытых линий.
    """
    segments: List[ProjectedSegment2D] = []
    if shape is None or shape.IsNull():
        return segments

    projector = HLRAlgo_Projector(view_ax2)

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr = HLRBRep_HLRToShape(algo)

    visible_comps = [
        hlr.OutLineVCompound(),  # внешние
        hlr.VCompound(),         # остальные видимые
    ]

    for comp in visible_comps:
        if comp is None or comp.IsNull():
            continue
        exp = TopExp_Explorer(comp, TopAbs_EDGE)
        while exp.More():
            edge = topods.Edge(exp.Current())
            try:
                for x1, y1, x2, y2 in _edge_to_poly_segments_2d(edge):
                    segments.append(
                        ProjectedSegment2D(
                            x1=x1, y1=y1,
                            x2=x2, y2=y2,
                            visible=True,
                        )
                    )
            except Exception:
                pass
            exp.Next()

    return segments

def build_linear_view(shape, view_ax2) -> List[ProjectedSegment2D]:
    """
    Сырой вид:
    - HLR в Ax2 вида;
    - используем V* и H* компаунды;
    - кривые дискретизируем с ограниченным числом сегментов.
    """
    segments: List[ProjectedSegment2D] = []
    if shape is None or shape.IsNull():
        return segments

    projector = HLRAlgo_Projector(view_ax2)

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr = HLRBRep_HLRToShape(algo)

    visible_comps = [
        hlr.OutLineVCompound(),
        hlr.VCompound(),
        hlr.Rg1LineVCompound(),
        hlr.RgNLineVCompound(),
    ]

    hidden_comps = [
        hlr.OutLineHCompound(),
        hlr.HCompound(),
        hlr.Rg1LineHCompound(),
        hlr.RgNLineHCompound(),
    ]

    # --- видимые сегменты ---
    for comp in visible_comps:
        if comp is None or comp.IsNull():
            continue
        exp = TopExp_Explorer(comp, TopAbs_EDGE)
        while exp.More():
            edge = topods.Edge(exp.Current())
            try:
                for x1, y1, x2, y2 in _edge_to_poly_segments_2d(edge):
                    segments.append(
                        ProjectedSegment2D(
                            x1=x1, y1=y1,
                            x2=x2, y2=y2,
                            visible=True,
                        )
                    )
            except Exception:
                pass
            exp.Next()

    # --- скрытые сегменты ---
    for comp in hidden_comps:
        if comp is None or comp.IsNull():
            continue
        exp = TopExp_Explorer(comp, TopAbs_EDGE)
        while exp.More():
            edge = topods.Edge(exp.Current())
            try:
                for x1, y1, x2, y2 in _edge_to_poly_segments_2d(edge):
                    segments.append(
                        ProjectedSegment2D(
                            x1=x1, y1=y1,
                            x2=x2, y2=y2,
                            visible=False,   # ← ключевое отличие
                        )
                    )
            except Exception:
                pass
            exp.Next()

    return segments


def get_bounds(*seg_lists):
    xs, ys = [], []
    for segs in seg_lists:
        for s in segs:
            xs.extend([s.x1, s.x2])
            ys.extend([s.y1, s.y2])
    return min(xs), max(xs), min(ys), max(ys)

def plot_view(ax, segments, min_x, max_x, max_y, min_y, title: str):
    for s in segments:
        ax.plot([s.x1, s.x2], [s.y1, s.y2],
                "-" if s.visible else "--", linewidth=1.0)
    ax.set_aspect("equal", "box")
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(max_y, min_y)  # инверсия Y, как на чертеже
    ax.set_title(title, fontsize=10)
    ax.grid(True, linestyle=":", linewidth=0.3)

def plot_base_drafting(shape):
    view_set = select_main_views(shape)
    ax_f = view_set.front.ax2
    ax_t = view_set.top.ax2
    ax_s = view_set.side.ax2
    front_segs = build_linear_view_debug(shape, ax_f)
    top_segs   = build_linear_view_debug(shape, ax_t)
    side_segs  = build_linear_view_debug(shape, ax_s)

    min_x, max_x, min_y, max_y = get_bounds(front_segs, top_segs, side_segs)

    fig, axes = plt.subplots(2, 2, figsize=(16, 8))

    plot_view(axes[0, 0], front_segs, min_x, max_x, min_y, max_y, "Front")
    plot_view(axes[1, 0], top_segs, min_x, max_x, min_y, max_y, "Top")
    plot_view(axes[0, 1], side_segs, min_x, max_x, min_y, max_y, "Side")

    axes[1, 1].axis("off")  # пустой квадрат

    plt.tight_layout()
    plt.show()