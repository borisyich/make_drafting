from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import math

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.BRepBndLib import brepbndlib_AddOBB
from OCC.Core.gp import gp_Dir, gp_Pnt, gp_Ax2, gp_Vec
from OCC.Core.HLRAlgo import HLRAlgo_Projector
from OCC.Core.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.GProp import GProp_GProps

from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopoDS import topods_Edge
import math

# --- Utility dataclasses ---

@dataclass
class ViewHLRStats:
    axis_index: int            # 0 -> d1, 1 -> d2, 2 -> d3
    sign: int                  # +1 или -1
    direction: gp_Dir          # нормаль взгляда в мировых координатах
    A_obb: float               # "объёмная" площадь проекции
    L_vis: float               # суммарная длина видимых линий
    L_hid: float               # суммарная длина скрытых линий

    @property
    def R_hidden(self) -> float:
        eps = 1e-9
        denom = self.L_vis + self.L_hid + eps
        return self.L_hid / denom


@dataclass
class OrthoViewDef:
    name: str                  # 'front'/'top'/'side'
    view_dir: gp_Dir           # направление взгляда (в мировых коорд.)
    u: gp_Dir                  # ось "вправо" в плоскости вида
    v: gp_Dir                  # ось "вверх" в плоскости вида


@dataclass
class ViewSet:
    front: OrthoViewDef
    top: OrthoViewDef
    side: OrthoViewDef
    # базис объекта (для шага 3 — построение 2D координат)
    x_obj: gp_Dir
    y_obj: gp_Dir
    z_obj: gp_Dir


@dataclass
class ProjectedSegment2D:
    x1: float
    y1: float
    x2: float
    y2: float
    visible: bool

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
        edge = topods_Edge(exp.Current())
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


def _compute_obb(shape) -> Tuple[gp_Pnt, Tuple[gp_Dir, gp_Dir, gp_Dir], Tuple[float, float, float]]:
    """
    Строит OBB для shape.
    Возвращает:
        center: gp_Pnt
        axes: (d1, d2, d3) как gp_Dir
        sizes: (sx, sy, sz) — полные размеры по этим осям
    """
    obb = Bnd_OBB()
    brepbndlib_AddOBB(shape, obb, True, True, True)

    center = obb.Center()
    d1 = obb.XDirection()
    d2 = obb.YDirection()
    d3 = obb.ZDirection()

    # XHSize/YHSize/ZHSize — половинные размеры, умножаем на 2
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


def select_main_views(
    shape, 
    hidden_weight_tol: float = 0.05
) -> ViewSet:
    """
    Реализует описанный тобой алгоритм Шага 2:
    - front: макс. A_obb, при близких A_obb — мин. R_hidden;
    - top/side: из оставшихся осей, ортогональных front.
    Возвращает ViewSet (front/top/side + базис объекта).
    """
    # 1) Все кандидаты ±d1, ±d2, ±d3
    candidates = analyze_view_candidates(shape)

    # 2) Выбор front
    # Сортируем по: сначала большая A_obb, потом меньший R_hidden
    sorted_for_front = sorted(
        candidates,
        key=lambda c: (-c.A_obb, c.R_hidden)
    )

    front_cand = sorted_for_front[0]

    # 3) Остальные две оси — кандидаты на top/side
    remaining_axis_indices = {0, 1, 2} - {front_cand.axis_index}

    # Для каждой оставшейся оси выбираем знак с лучшим (меньшим) R_hidden
    axis_best: dict[int, ViewHLRStats] = {}
    for idx in remaining_axis_indices:
        cand_for_axis = [c for c in candidates if c.axis_index == idx]
        best = min(cand_for_axis, key=lambda c: c.R_hidden)
        axis_best[idx] = best

    # Теперь нужно выбрать top: среди двух осей выбираем с макс. A_obb,
    # при близких A_obb используем R_hidden.
    axis_items = list(axis_best.items())
    (axis_idx1, cand1), (axis_idx2, cand2) = axis_items

    def _score_for_top(c: ViewHLRStats):
        return (-c.A_obb, c.R_hidden)

    # сравниваем
    if _score_for_top(cand1) <= _score_for_top(cand2):
        top_cand = cand1
        side_cand = cand2
    else:
        top_cand = cand2
        side_cand = cand1

    # 4) Строим базис объекта.
    # z_obj = -n_f, y_obj = -n_t, x_obj = normalize(y_obj × z_obj)
    n_f = front_cand.direction   # нормаль front
    n_t = top_cand.direction     # нормаль top

    z_obj = _normalize_vec(-n_f.X(), -n_f.Y(), -n_f.Z())
    y_obj = _normalize_vec(-n_t.X(), -n_t.Y(), -n_t.Z())

    # векторное произведение y_obj × z_obj
    cx = y_obj.Y() * z_obj.Z() - y_obj.Z() * z_obj.Y()
    cy = y_obj.Z() * z_obj.X() - y_obj.X() * z_obj.Z()
    cz = y_obj.X() * z_obj.Y() - y_obj.Y() * z_obj.X()
    x_obj = _normalize_vec(cx, cy, cz)

    # 5) Определяем направления вида и оси u/v для трёх стандартных видов

    # Front:
    # смотрим вдоль -z_obj (в сторону детали)
    view_dir_front = _normalize_vec(-z_obj.X(), -z_obj.Y(), -z_obj.Z())
    u_f = x_obj            # вправо
    v_f = y_obj            # вверх

    # Top:
    # смотрим вдоль -y_obj (сверху вниз)
    view_dir_top = _normalize_vec(-y_obj.X(), -y_obj.Y(), -y_obj.Z())
    u_t = x_obj            # вправо (как у фронта)
    # вверх экрана — по -z_obj, чтобы топ был "над" фронтом
    v_t = _normalize_vec(-z_obj.X(), -z_obj.Y(), -z_obj.Z())

    # Side (правый вид):
    # смотрим вдоль -x_obj (справа налево)
    view_dir_side = _normalize_vec(-x_obj.X(), -x_obj.Y(), -x_obj.Z())
    # вправо на чертеже — по z_obj, чтобы правый вид был справа от фронта
    u_s = z_obj
    # вверх — y_obj (как у фронта)
    v_s = y_obj

    front_view = OrthoViewDef(
        name="front",
        view_dir=view_dir_front,
        u=u_f,
        v=v_f,
    )

    top_view = OrthoViewDef(
        name="top",
        view_dir=view_dir_top,
        u=u_t,
        v=v_t,
    )

    side_view = OrthoViewDef(
        name="side",
        view_dir=view_dir_side,
        u=u_s,
        v=v_s,
    )

    return ViewSet(
        front=front_view,
        top=top_view,
        side=side_view,
        x_obj=x_obj,
        y_obj=y_obj,
        z_obj=z_obj,
    )

def _compute_obb_center(shape) -> gp_Pnt:
    obb = Bnd_OBB()
    brepbndlib_AddOBB(shape, obb, True, True, True)
    return obb.Center()


def _project_point_to_uv(p: gp_Pnt, origin: gp_Pnt, u: gp_Dir, v: gp_Dir) -> tuple[float, float]:
    """
    Проекция 3D-точки p в 2D-координаты (ξ, η) в базисе (u, v) с центром origin.
    Вариант БЕЗ gp_Vec, чтобы не ловить problemer с перегрузками в 7.9.0.
    """
    # вектор от origin к p как просто разность координат
    vx = p.X() - origin.X()
    vy = p.Y() - origin.Y()
    vz = p.Z() - origin.Z()

    # скалярное произведение с направлениями u и v
    xi  = vx * u.X() + vy * u.Y() + vz * u.Z()
    eta = vx * v.X() + vy * v.Y() + vz * v.Z()

    return float(xi), float(eta)

def build_linear_view(shape, view: OrthoViewDef) -> List[ProjectedSegment2D]:
    """
    Строит "сырой" линейный вид:
    - запускает HLR в направлении view.view_dir;
    - разбирает видимые/скрытые рёбра;
    - каждое ребро аппроксимирует 2D-отрезком в базисе (view.u, view.v).

    Это дебажная версия: один сегмент = отрезок между концами ребра.
    """
    segments: List[ProjectedSegment2D] = []

    if shape is None or shape.IsNull():
        return segments

    # Используем центр OBB как "центр сцены" для проекции на (u,v)
    origin = _compute_obb_center(shape)

    # 1) HLR в заданном направлении взгляда
    projector = HLRAlgo_Projector(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), view.view_dir))

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr_shapes = HLRBRep_HLRToShape(algo)

    visible_compounds = [
        hlr_shapes.VCompound(),
        hlr_shapes.OutLineVCompound(),
        hlr_shapes.Rg1LineVCompound(),
        hlr_shapes.RgNLineVCompound(),
    ]
    hidden_compounds = [
        hlr_shapes.HCompound(),
        hlr_shapes.OutLineHCompound(),
        hlr_shapes.Rg1LineHCompound(),
        hlr_shapes.RgNLineHCompound(),
    ]

    def _extract_segments_from_compounds(compounds, visible_flag: bool):
        for comp in compounds:
            if comp is None or comp.IsNull():
                continue
            exp = TopExp_Explorer(comp, TopAbs_EDGE)
            while exp.More():
                edge = topods_Edge(exp.Current())
                try:
                    curve = BRepAdaptor_Curve(edge)
                    first = curve.FirstParameter()
                    last = curve.LastParameter()

                    if not math.isfinite(first) or not math.isfinite(last) or last <= first:
                        exp.Next()
                        continue

                    p1 = curve.Value(first)
                    p2 = curve.Value(last)

                    x1, y1 = _project_point_to_uv(p1, origin, view.u, view.v)
                    x2, y2 = _project_point_to_uv(p2, origin, view.u, view.v)

                    segments.append(
                        ProjectedSegment2D(
                            x1=x1,
                            y1=y1,
                            x2=x2,
                            y2=y2,
                            visible=visible_flag,
                        )
                    )
                except Exception as e:
                    print(e)
                    # Не роняем процесс из-за одного проблемного ребра
                    pass

                exp.Next()

    _extract_segments_from_compounds(visible_compounds, True)
    _extract_segments_from_compounds(hidden_compounds, False)

    return segments