from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple, Set

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.gp import gp_Ax2, gp_Pnt, gp_Vec
from OCC.Core.Poly import Poly_Triangle
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import topexp as TopExp, TopExp_Explorer
from OCC.Core.TopTools import TopTools_IndexedMapOfShape
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Face, topods_Face, topods
from OCC.Core.TopLoc import TopLoc_Location


@dataclass
class FaceVisibilityResult:
    """
    Результат видимости граней для одного вида.

    Attributes
    ----------
    face_map:
        IndexedMapOfShape, содержащая все грани исходного shape.
        Индексы граней (1..N) используются как face_index.
    visible_faces:
        Множество индексов граней (1..face_map.Extent()), которые считаются
        видимыми на данном виде.
    """

    face_map: TopTools_IndexedMapOfShape
    visible_faces: Set[int]

    def is_face_visible(self, face: TopoDS_Face) -> bool:
        """Проверка видимости конкретной грани по её объекту."""
        idx = self.face_map.FindIndex(face)
        if idx == 0:
            return False
        return idx in self.visible_faces

    def is_index_visible(self, face_index: int) -> bool:
        """Проверка видимости по индексу 1..face_map.Extent()."""
        return face_index in self.visible_faces


@dataclass
class MultiViewFaceVisibility:
    """
    Результат видимости граней для нескольких видов.

    Attributes
    ----------
    face_map:
        Общий IndexedMapOfShape по всем видам (один на исходный shape).
    visible_on_view:
        Словарь view_name -> множество индексов видимых граней на этом виде.
    """

    face_map: TopTools_IndexedMapOfShape
    visible_on_view: Dict[str, Set[int]]

    def views_for_face_index(self, face_index: int) -> Set[str]:
        """
        На каких видах эта грань видима (по индексу)?
        """
        return {name for name, vis in self.visible_on_view.items()
                if face_index in vis}

    def views_for_face(self, face: TopoDS_Face) -> Set[str]:
        """
        На каких видах видна данная грань (по объекту TopoDS_Face)?
        """
        idx = self.face_map.FindIndex(face)
        if idx == 0:
            return set()
        return self.views_for_face_index(idx)


# ---------------------------------------------------------------------------
# Внутренние вспомогательные функции
# ---------------------------------------------------------------------------

def _build_face_map(shape: TopoDS_Shape) -> TopTools_IndexedMapOfShape:
    """
    Строит IndexedMapOfShape для всех граней исходного shape.
    Индексация 1..N.

    Это даёт стабильные целочисленные индексы для границ,
    чтобы не полагаться на hash(face).
    """
    face_map = TopTools_IndexedMapOfShape()
    TopExp.MapShapes(shape, TopAbs_FACE, face_map)
    return face_map


def _project_point_to_view(pnt: gp_Pnt, ax2: gp_Ax2) -> Tuple[float, float, float]:
    """
    Проецирует 3D-точку в координаты системы вида (ξ, η, ζ):

        ξ  — по XDirection(),
        η  — по YDirection(),
        ζ  — по Direction() (ось взгляда).

    ζ используется как "глубина" для z-buffer:
        меньше ζ -> ближе к наблюдателю
    (при условии, что направление взгляда выбрано согласованно).
    """
    origin = ax2.Location()
    vx = gp_Vec(ax2.XDirection())
    vy = gp_Vec(ax2.YDirection())
    vz = gp_Vec(ax2.Direction())

    v = gp_Vec(origin, pnt)
    xi = vx.Dot(v)
    eta = vy.Dot(v)
    zeta = vz.Dot(v)

    return xi, eta, zeta


# ---------------------------------------------------------------------------
# Основной алгоритм z-buffer по триангуляции
# ---------------------------------------------------------------------------

def compute_face_visibility_by_zbuffer(
    shape: TopoDS_Shape,
    view_ax2: gp_Ax2,
    *,
    grid_size: int = 256,
    deflection: float = 5e-4,
    angle: float = 0.5,
) -> FaceVisibilityResult:
    """
    Приближённо определяет, какие грани shape видимы на заданном виде
    (ортогональная проекция в системе координат view_ax2) с помощью
    грубого z-buffer по центрам триангуляции.

    Параметры
    ---------
    shape:
        Исходный TopoDS_Shape (сплошное тело или сборка).
    view_ax2:
        gp_Ax2, задающая систему координат вида:
            Direction()  -> направление взгляда,
            XDirection() -> ось ξ,
            YDirection() -> ось η.
    grid_size:
        Разрешение регулярной сетки в плоскости вида (ξ, η) для z-buffer.
        256..512 обычно достаточно. Чем больше, тем точнее и медленнее.
    deflection:
        Параметр триангуляции (BRepMesh_IncrementalMesh), допуск по отклонению.
    angle:
        Максимальное отклонение нормали в радианах (так же для мезера).

    Возвращает
    ----------
    FaceVisibilityResult:
        - face_map: IndexedMapOfShape (все грани shape).
        - visible_faces: множество индексов видимых граней (1..N).
    """

    # --- 1. Строим карту граней ---
    face_map = _build_face_map(shape)
    nb_faces = face_map.Size()
    if nb_faces == 0:
        return FaceVisibilityResult(face_map=face_map, visible_faces=set())

    # --- 2. Триангулируем shape (если уже триангулирован — OCCT переприведёт как надо) ---
    BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)

    # --- 3. Собираем центры треугольников в координатах вида ---
    tris = []  # список (face_index, xi, eta, zeta)
    xis = []
    etas = []

    # Для корректного получения triangulation учитываем возможные location
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        exp.Next()

        face_index = face_map.FindIndex(face)
        if face_index == 0:
            # На всякий случай, но по идее все грани должны быть в face_map
            continue

        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation(face, loc)
        if tri is None:
            continue

        nb_tris = tri.NbTriangles()

        # Преобразование по location, если оно задано
        has_loc = not loc.IsIdentity()
        trsf = loc.Transformation() if has_loc else None

        for i_tri in range(1, nb_tris + 1):
            poly_tri: Poly_Triangle = tri.Triangle(i_tri)
            n1, n2, n3 = poly_tri.Get()

            p1 = tri.Node(n1)
            p2 = tri.Node(n2)
            p3 = tri.Node(n3)

            if has_loc:
                p1.Transform(trsf)
                p2.Transform(trsf)
                p3.Transform(trsf)

            cx = (p1.X() + p2.X() + p3.X()) / 3.0
            cy = (p1.Y() + p2.Y() + p3.Y()) / 3.0
            cz = (p1.Z() + p2.Z() + p3.Z()) / 3.0
            pc = gp_Pnt(cx, cy, cz)

            xi, eta, zeta = _project_point_to_view(pc, view_ax2)

            tris.append((face_index, xi, eta, zeta))
            xis.append(xi)
            etas.append(eta)


    if not tris:
        # Ни одной триангулированной грани
        return FaceVisibilityResult(face_map=face_map, visible_faces=set())

    # --- 4. Строим bounding box в (ξ, η) + небольшой паддинг ---
    xi_min, xi_max = min(xis), max(xis)
    eta_min, eta_max = min(etas), max(etas)

    dx = xi_max - xi_min
    dy = eta_max - eta_min
    # защита от вырожденности
    if dx == 0.0:
        dx = 1.0
    if dy == 0.0:
        dy = 1.0

    pad_x = 0.01 * dx
    pad_y = 0.01 * dy

    xi_min -= pad_x
    xi_max += pad_x
    eta_min -= pad_y
    eta_max += pad_y

    # --- 5. Z-buffer: (ix, iy) -> (zeta, face_index) ---
    zbuffer: Dict[Tuple[int, int], Tuple[float, int]] = {}

    def to_grid(xi: float, eta: float) -> Tuple[int, int]:
        gx = (xi - xi_min) / (xi_max - xi_min)
        gy = (eta - eta_min) / (eta_max - eta_min)
        ix = int(gx * (grid_size - 1))
        iy = int(gy * (grid_size - 1))
        # На всякий случай ограничим диапазон
        ix = max(0, min(grid_size - 1, ix))
        iy = max(0, min(grid_size - 1, iy))
        return ix, iy

    for face_index, xi, eta, zeta in tris:
        ix, iy = to_grid(xi, eta)
        key = (ix, iy)

        # Выбираем минимальный zeta как "ближе к наблюдателю"
        if key not in zbuffer:
            zbuffer[key] = (zeta, face_index)
        else:
            z_old, f_old = zbuffer[key]
            if zeta < z_old:
                zbuffer[key] = (zeta, face_index)

    # --- 6. Собираем множество видимых граней ---
    visible_faces: Set[int] = set()
    for (_ix, _iy), (_z, f_index) in zbuffer.items():
        visible_faces.add(f_index)

    return FaceVisibilityResult(face_map=face_map, visible_faces=visible_faces)


def compute_multiview_face_visibility(
    shape: TopoDS_Shape,
    views: Dict[str, gp_Ax2],
    *,
    grid_size: int = 256,
    deflection: float = 5e-4,
    angle: float = 0.5,
) -> MultiViewFaceVisibility:
    """
    Вычисляет видимость граней shape для нескольких видов сразу.

    Параметры
    ---------
    shape:
        Исходный shape.
    views:
        Словарь имя_вида -> gp_Ax2 (система координат вида).
        Например:
            {
              "front": ax2_front,
              "top": ax2_top,
              "side": ax2_side,
            }
    grid_size, deflection, angle:
        Параметры z-buffer и триангуляции (см. compute_face_visibility_by_zbuffer).

    Возвращает
    ----------
    MultiViewFaceVisibility:
        - общий face_map,
        - словарь view_name -> множество видимых индексов граней.
    """

    # Общая карта граней для всех видов
    face_map = _build_face_map(shape)
    visible_on_view: Dict[str, Set[int]] = {}

    for name, ax2 in views.items():
        res = compute_face_visibility_by_zbuffer(
            shape,
            ax2,
            grid_size=grid_size,
            deflection=deflection,
            angle=angle,
        )
        # На всякий случай можно проверить, что res.face_map совместим с нашим face_map,
        # но так как _build_face_map детерминированно, здесь считаем, что он совпадает.
        visible_on_view[name] = res.visible_faces

    return MultiViewFaceVisibility(face_map=face_map, visible_on_view=visible_on_view)

