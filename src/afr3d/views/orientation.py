"""Orientation helpers placeholder for future view selection logic."""

# TODO: implement oriented bounding boxes and base axis selection.

import math

from dataclasses import dataclass
from typing import Dict, List, Tuple

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Ax2
from OCC.Core.TopoDS import TopoDS_Shape

from OCC.Core.gp import gp_Dir, gp_Pnt, gp_Ax2
from OCC.Core.HLRAlgo import HLRAlgo_Projector
from OCC.Core.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape

from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint

from OCC.Core.TopoDS import topods

def to_edge(shape):
    return topods.Edge(shape)

from afr3d.drafting.model import ViewHLRStats, OrthoViewDef, ViewSet

@dataclass
class OrientedAxes:
    origin: gp_Pnt
    d1: gp_Dir
    d2: gp_Dir
    d3: gp_Dir

def compute_obb(shape):
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

def compute_obb_axes(shape) -> OrientedAxes:
    """
    Compute obb wrap
    """
    center, axes, sizes = compute_obb(shape)
    center_pnt = gp_Pnt(center.X(), center.Y(), center.Z())
    return OrientedAxes(
        origin=center_pnt,
        d1=axes[0],
        d2=axes[1],
        d3=axes[2]
    )


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


def _normalize_vec(x: float, y: float, z: float) -> gp_Dir:
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("Zero-length vector in _normalize_vec")
    return gp_Dir(x / norm, y / norm, z / norm)


def _gpdir_scaled(dir_: gp_Dir, sign: int) -> gp_Dir:
    """Возвращает gp_Dir с тем же направлением, но с нужным знаком."""
    return gp_Dir(sign * dir_.X(), sign * dir_.Y(), sign * dir_.Z())


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


def analyze_view_candidates(shape) -> List[ViewHLRStats]:
    """
    Строит OBB и считает HLR-метрики для шести направлений ±d1, ±d2, ±d3.
    Возвращает список ViewHLRStats.
    """
    _, (d1, d2, d3), (sx, sy, sz) = compute_obb(shape)

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
    center, _, _ = compute_obb(shape)
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

def make_default_front_top_side_ax2(shape: TopoDS_Shape) -> Dict[str, gp_Ax2]:
    """
    Простейший выбор front/top/side из осей OBB.
    Нормали:
      front ~ -d3,
      top   ~ -d2,
      side  ~ +d1.
    """
    main_views = select_main_views(shape)

    return {
        "front": main_views.front.ax2,
        "top":   main_views.top.ax2,
        "side":  main_views.side.ax2,
    }