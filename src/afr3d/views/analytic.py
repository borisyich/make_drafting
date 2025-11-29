"""
afr3d.views.analytic

Аналитическое представление вида:
  - 2D-вершины и 2D-рёбра в координатах вида (ξ, η)
  - связь с 3D-топологией (vertex_index, edge_index, face_index)
  - оценка видимости рёбер и вершин по видимости граней (z-buffer)

Зависит от:
  - afr3d.views.visibility.FaceVisibilityResult (или MultiViewFaceVisibility)
  - gp_Ax2 вида (та же система, что для HLR)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Set, Literal

from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Ax2
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.GeomAbs import (
    GeomAbs_Line,
    GeomAbs_Circle,
    GeomAbs_Ellipse,
    GeomAbs_BSplineCurve,
    GeomAbs_BezierCurve,
    GeomAbs_OtherCurve,
    GeomAbs_Plane,
    GeomAbs_Cylinder,
    GeomAbs_Cone,
    GeomAbs_Sphere,
    GeomAbs_Torus,
    GeomAbs_BSplineSurface,
    GeomAbs_BezierSurface,
    GeomAbs_OtherSurface,
)
from OCC.Core.TopoDS import (
    TopoDS_Shape,
    TopoDS_Face,
    TopoDS_Edge,
    TopoDS_Vertex,
    topods
)
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer, topexp as TopExp
from OCC.Core.TopTools import TopTools_IndexedMapOfShape

from afr3d.views.visibility import FaceVisibilityResult, compute_face_visibility_by_zbuffer
from afr3d.views.orientation import make_default_front_top_side_ax2

# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------

def face_surface_type(face: TopoDS_Face) -> str:
    try:
        surf = BRepAdaptor_Surface(face, True)
        stype = surf.GetType()
    except Exception:
        return "unknown"

    mapping = {
        GeomAbs_Plane: "plane",
        GeomAbs_Cylinder: "cylinder",
        GeomAbs_Cone: "cone",
        GeomAbs_Sphere: "sphere",
        GeomAbs_Torus: "torus",
        GeomAbs_BSplineSurface: "bspline",
        GeomAbs_BezierSurface: "bezier",
        GeomAbs_OtherSurface: "other",
    }
    return mapping.get(stype, f"unknown({int(stype)})")

# “Нативный” подсчёт граней из shape
def count_faces_native(shape):
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    n = 0
    while exp.More():
        n += 1
        exp.Next()
    return n

def count_edges_native(shape):
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    n = 0
    while exp.More():
        n += 1
        exp.Next()
    return n


def _map_extent(m) -> int:
    """Безопасно получить размер IndexedMapOfShape для разных версий pythonocc."""
    if hasattr(m, "Extent"):
        return m.Extent()
    if hasattr(m, "Size"):
        return m.Size()
    n = 0
    try:
        while True:
            n += 1
            m.FindKey(n)
    except Exception:
        pass
    return n


def project_point_to_view_2d(pnt: gp_Pnt, ax2: gp_Ax2) -> Tuple[float, float]:
    """
    3D → 2D в координатах вида:
      ξ  вдоль XDirection()
      η  вдоль YDirection()
    Используем для проекции ИСХОДНОЙ геометрии (граней/рёбер/вершин)
    в ту же плоскость, где живут HLR-результаты.
    """
    origin = ax2.Location()
    vx = gp_Vec(ax2.XDirection())
    vy = gp_Vec(ax2.YDirection())
    v = gp_Vec(origin, pnt)
    xi = vx.Dot(v)
    eta = vy.Dot(v)
    return xi, eta


def curve_type_name(geom_type: int) -> str:
    mapping = {
        GeomAbs_Line: "line",
        GeomAbs_Circle: "circle",
        GeomAbs_Ellipse: "ellipse",
        GeomAbs_BSplineCurve: "bspline",
        GeomAbs_BezierCurve: "bezier",
        GeomAbs_OtherCurve: "other",
    }
    return mapping.get(geom_type, f"unknown({int(geom_type)})")


# ---------------------------------------------------------------------------
# Структуры аналитического вида
# ---------------------------------------------------------------------------

@dataclass
class DraftFaceInfo:
    face_index: int
    face_3d: TopoDS_Face
    surface_type: str         # plane/cylinder/...
    visible: bool = False     # видна ли по z-buffer


@dataclass
class DraftVertex2D:
    """
    2D-представление вершины на виде.

    vertex_index:
        индекс вершины в vertex_map (1..N)
    x, y:
        координаты в плоскости вида (ξ, η)
    incident_edges:
        индексы рёбер (edge_index), которые к ней подключены
    visible:
        считается видимой, если есть хотя бы одно видимое ребро
    """

    vertex_index: int
    x: float
    y: float
    vertex_3d: TopoDS_Vertex
    incident_edges: List[int] = field(default_factory=list)
    visible: bool = False


@dataclass
class DraftEdge2D:
    """
    2D-представление ребра на виде.

    edge_index:
        индекс ребра в edge_map (1..M)
    v_start, v_end:
        индексы вершин (vertex_index) на концах ребра
    points:
        дискретизация 2D-кривой (полилиния) в координатах вида
    face_indices:
        индексы граней (face_index), которым принадлежит ребро (1 или 2)
    curve_type:
        тип кривой (line/circle/ellipse/bspline/...)
    visibility:
        'visible', 'silhouette', 'hidden', 'unknown'
    line_style:
        стиль линии (solid/hidden/center/thick_silhouette/...)
    surface_types:
        типы поверхностей по обе стороны
    """

    edge_index: int
    v_start: Optional[int]
    v_end: Optional[int]
    points: List[Tuple[float, float]]
    edge_3d: TopoDS_Edge
    face_indices: List[int]
    curve_type: str
    visibility: str = "unknown"

    line_style: str = "default"      # solid/hidden/center/thick_silhouette/...
    surface_types: List[str] = field(default_factory=list)  # типы поверхностей по обе стороны



@dataclass
class AnalyticView2D:
    """
    Полное аналитическое описание одного вида.

    view_name:
        имя вида ("front", "top", "side", "section_A" и т.п.)
    ax2:
        gp_Ax2 системы координат вида.
    vertex_map, edge_map, face_map:
        карты TopoDS-объектов в индексы 1..N
    vertices:
        индекс вершины -> DraftVertex2D
    edges:
        индекс ребра -> DraftEdge2D
    """

    view_name: str
    ax2: gp_Ax2

    vertex_map: TopTools_IndexedMapOfShape
    edge_map: TopTools_IndexedMapOfShape
    face_map: TopTools_IndexedMapOfShape

    vertices: Dict[int, DraftVertex2D]
    edges: Dict[int, DraftEdge2D]

    faces: Dict[int, DraftFaceInfo]

# ---------------------------------------------------------------------------
# Фильтры
# ---------------------------------------------------------------------------

VisibilityFilter = Literal[
    "all",
    "visible_only",
    "hidden_only",
    "visible_plus_silhouette",
]


def filter_edges_by_visibility(view: AnalyticView2D, mode: VisibilityFilter):
    """
    Возвращает список рёбер для рисования в зависимости от фильтра видимости.
    """
    if mode == "all":
        return list(view.edges.values())
    elif mode == "visible_only":
        return [e for e in view.edges.values() if e.visibility == "visible"]
    elif mode == "hidden_only":
        return [e for e in view.edges.values() if e.visibility == "hidden"]
    elif mode == "visible_plus_silhouette":
        return [e for e in view.edges.values() if e.visibility in ("visible", "silhouette")]
    else:
        raise ValueError(f"Unknown visibility mode: {mode}")
    
# ---------------------------------------------------------------------------
# Построение топологии: карты вершин/рёбер/граней и инцидентность
# ---------------------------------------------------------------------------

def build_topology_maps(shape: TopoDS_Shape):
    """
    Строит:
      - vertex_map: все вершины
      - edge_map:   все рёбра
      - face_map:   все грани
      - edge_to_vertices: edge_index -> (v_start_idx, v_end_idx)
      - edge_to_faces:    edge_index -> [face_index1, face_index2, ...]

    Индексы 1..N для всех карт.
    """
    vertex_map = TopTools_IndexedMapOfShape()
    edge_map = TopTools_IndexedMapOfShape()
    face_map = TopTools_IndexedMapOfShape()

    TopExp.MapShapes(shape, TopAbs_VERTEX, vertex_map)
    TopExp.MapShapes(shape, TopAbs_EDGE, edge_map)
    TopExp.MapShapes(shape, TopAbs_FACE, face_map)

    n_edges = _map_extent(edge_map)
    n_faces = _map_extent(face_map)

    # edge -> (v_start, v_end)
    edge_to_vertices: Dict[int, Tuple[Optional[int], Optional[int]]] = {}

    for e_idx in range(1, n_edges + 1):
        edge = topods.Edge(edge_map.FindKey(e_idx))

        v1 = TopExp.FirstVertex(edge, True)   # True -> ориентация учитывается
        v2 = TopExp.LastVertex(edge, True)

        v1_idx = vertex_map.FindIndex(v1) if not v1.IsNull() else None
        v2_idx = vertex_map.FindIndex(v2) if not v2.IsNull() else None

        edge_to_vertices[e_idx] = (v1_idx, v2_idx)

    # edge -> faces
    edge_to_faces: Dict[int, List[int]] = {e_idx: [] for e_idx in range(1, n_edges + 1)}

    for f_idx in range(1, n_faces + 1):
        face = topods.Face(face_map.FindKey(f_idx))
        exp = TopExp_Explorer(face, TopAbs_EDGE)
        while exp.More():
            edge = topods.Edge(exp.Current())
            exp.Next()

            e_idx = edge_map.FindIndex(edge)
            if e_idx == 0:
                continue
            edge_to_faces[e_idx].append(f_idx)

    return vertex_map, edge_map, face_map, edge_to_vertices, edge_to_faces


# ---------------------------------------------------------------------------
# Проекция вершин и рёбер в 2D
# ---------------------------------------------------------------------------

def collect_face_infos(face_map, face_visibility: Optional["FaceVisibilityResult"]) -> Dict[int, DraftFaceInfo]:
    n_faces = _map_extent(face_map)
    infos: Dict[int, DraftFaceInfo] = {}
    for f_idx in range(1, n_faces + 1):
        face = topods.Face(face_map.FindKey(f_idx))
        stype = face_surface_type(face)
        vis = face_visibility.is_index_visible(f_idx) if face_visibility is not None else False
        infos[f_idx] = DraftFaceInfo(
            face_index=f_idx,
            face_3d=face,
            surface_type=stype,
            visible=vis,
        )
    return infos


def collect_projected_vertices(
    vertex_map: TopTools_IndexedMapOfShape,
    view_ax2: gp_Ax2,
) -> Dict[int, DraftVertex2D]:
    """
    Проецирует все вершины vertex_map в 2D и возвращает словарь:
      vertex_index -> DraftVertex2D
    """
    proj_vertices: Dict[int, DraftVertex2D] = {}

    n_vertices = _map_extent(vertex_map)
    for v_idx in range(1, n_vertices + 1):
        vtx = topods.Vertex(vertex_map.FindKey(v_idx))
        p3d = BRep_Tool.Pnt(vtx)
        x, y = project_point_to_view_2d(p3d, view_ax2)
        proj_vertices[v_idx] = DraftVertex2D(
            vertex_index=v_idx,
            x=x,
            y=y,
            vertex_3d=vtx,
        )

    return proj_vertices


def sample_edge_2d_from_shape(edge: TopoDS_Edge, ax2: gp_Ax2, n_samples: int = 32) -> List[Tuple[float, float]]:
    """
    Семплирует ребро исходного shape и возвращает точки в координатах вида.
    """
    pts_2d: List[Tuple[float, float]] = []

    try:
        bac = BRepAdaptor_Curve(edge)
    except Exception:
        return pts_2d

    try:
        first = bac.FirstParameter()
        last = bac.LastParameter()
    except Exception:
        return pts_2d

    if last <= first:
        return pts_2d

    if bac.GetType() == GeomAbs_Line:
        params = [first, last]
    else:
        params = [first + (last - first) * i / (n_samples - 1) for i in range(n_samples)]

    for u in params:
        try:
            p3d = bac.Value(u)
        except Exception:
            continue
        x, y = project_point_to_view_2d(p3d, ax2)
        pts_2d.append((x, y))

    return pts_2d


def collect_projected_edges(
    edge_map: TopTools_IndexedMapOfShape,
    edge_to_vertices: Dict[int, Tuple[Optional[int], Optional[int]]],
    edge_to_faces: Dict[int, List[int]],
    view_ax2: gp_Ax2,
    *,
    n_samples: int = 32,
) -> Dict[int, DraftEdge2D]:
    """
    Проецирует все рёбра edge_map в 2D и возвращает:
      edge_index -> DraftEdge2D
    """
    proj_edges: Dict[int, DraftEdge2D] = {}

    n_edges = _map_extent(edge_map)
    for e_idx in range(1, n_edges + 1):
        edge = topods.Edge(edge_map.FindKey(e_idx))
        pts = sample_edge_2d_from_shape(edge, view_ax2, n_samples=n_samples)

        # тип кривой
        curve_type = "unknown"
        try:
            bac = BRepAdaptor_Curve(edge)
            curve_type = curve_type_name(bac.GetType())
        except Exception:
            pass

        v_start, v_end = edge_to_vertices.get(e_idx, (None, None))
        faces = edge_to_faces.get(e_idx, [])

        proj_edges[e_idx] = DraftEdge2D(
            edge_index=e_idx,
            v_start=v_start,
            v_end=v_end,
            points=pts,
            edge_3d=edge,
            face_indices=faces,
            curve_type=curve_type,
            visibility="unknown",
        )

    return proj_edges


# ---------------------------------------------------------------------------
# Присвоение видимости рёбер и вершин по видимости граней
# ---------------------------------------------------------------------------

def assign_visibility_to_edges_and_vertices(
    proj_edges: Dict[int, DraftEdge2D],
    proj_vertices: Dict[int, DraftVertex2D],
    face_visibility: Optional["FaceVisibilityResult"] = None,
):
    """
    Обновляет поля visibility у рёбер и visible у вершин.

    Логика по рёбрам:
      - нет face_visibility -> всё 'unknown'
      - у ребра есть список face_indices:
          v_faces = те из них, которые видимы по face_visibility
          if len(v_faces) == 0: visibility = 'hidden'
          elif len(v_faces) == len(face_indices): visibility = 'visible'
          else: visibility = 'silhouette'

    Вершина считается visible, если к ней подключено хотя бы одно
    ребро с visibility in {'visible', 'silhouette'}.
    """
    if face_visibility is None:
        return

    # --- рёбра ---
    for e_idx, e in proj_edges.items():
        if not e.face_indices:
            e.visibility = "unknown"
            e.surface_types = []
            e.line_style = "default"


        visible_faces = [fi for fi in e.face_indices if face_visibility.is_index_visible(fi)]

        if not visible_faces:
            e.visibility = "hidden"
        elif len(visible_faces) == len(e.face_indices):
            e.visibility = "visible"
        else:
            e.visibility = "silhouette"
        
        # surface_types соседних граней
        # предполагаем, что у тебя есть словарь face_infos: face_index -> DraftFaceInfo
        # сюда можно передать его через параметр или добавить в AnalyticView2D после
        e.surface_types = []  # временно, позже заполним, когда есть face_infos

        # тип линии для чертежа
        if e.visibility == "visible":
            e.line_style = "solid"
        elif e.visibility == "hidden":
            e.line_style = "hidden"          # штриховая
        elif e.visibility == "silhouette":
            e.line_style = "solid_thick"     # утолщённая сплошная (контур детали)
        else:
            e.line_style = "default"

    # --- вершины: сначала очистим видимость и списки инцидентности ---
    for v in proj_vertices.values():
        v.incident_edges.clear()
        v.visible = False

    # строим инцидентность
    for e_idx, e in proj_edges.items():
        for v_idx in (e.v_start, e.v_end):
            if v_idx is None:
                continue
            if v_idx in proj_vertices:
                proj_vertices[v_idx].incident_edges.append(e_idx)

    # вершины видимы, если есть хотя бы одно видимое или silhouette ребро
    visible_edge_states = {"visible", "silhouette"}
    for v_idx, v in proj_vertices.items():
        for e_idx in v.incident_edges:
            if proj_edges[e_idx].visibility in visible_edge_states:
                v.visible = True
                break


# ---------------------------------------------------------------------------
# Высокоуровневая функция: построение аналитического вида
# ---------------------------------------------------------------------------

def build_analytic_view_2d(
    shape: TopoDS_Shape,
    view_ax2: gp_Ax2,
    view_name: str = "front",
    face_visibility: Optional["FaceVisibilityResult"] = None,
    *,
    n_edge_samples: int = 32,
) -> AnalyticView2D:
    """
    Строит полный аналитический вид (AnalyticView2D) для одного направления вида.

    Параметры
    ---------
    shape:
        исходный TopoDS_Shape
    view_ax2:
        gp_Ax2, задающая систему координат вида (как для HLR)
    view_name:
        имя вида ("front", "top", "side", "section_A", ...)
    face_visibility:
        результат compute_face_visibility_by_zbuffer для ЭТОГО вида.
        Если None, то visibility рёбер/вершин остаётся 'unknown'/False.
    n_edge_samples:
        количество точек дискретизации на кривой ребра (для полилинии).

    Возвращает
    ----------
    AnalyticView2D
    """
    # 1. Топология: карты и инцидентность
    vertex_map, edge_map, face_map, edge_to_vertices, edge_to_faces = build_topology_maps(shape)

    # 2. Проекция вершин и рёбер
    proj_vertices = collect_projected_vertices(vertex_map, view_ax2)
    proj_edges = collect_projected_edges(
        edge_map=edge_map,
        edge_to_vertices=edge_to_vertices,
        edge_to_faces=edge_to_faces,
        view_ax2=view_ax2,
        n_samples=n_edge_samples,
    )

    # 3. Видимость по граням (z-buffer)
    if face_visibility is not None:
        assign_visibility_to_edges_and_vertices(proj_edges, proj_vertices, face_visibility=face_visibility)

    # ИНФО!
        face_infos = collect_face_infos(face_map, face_visibility)
    else:
        face_infos = None

    # 4. Собираем AnalyticView2D
    view = AnalyticView2D(
        view_name=view_name,
        ax2=view_ax2,
        vertex_map=vertex_map,
        edge_map=edge_map,
        face_map=face_map,
        vertices=proj_vertices,
        edges=proj_edges,
        faces=face_infos
    )

    # заполняем surface_types
    for e in proj_edges.values():
        e.surface_types = [view.faces[fi].surface_type for fi in e.face_indices 
                           if fi in view.faces]
    
    return view


# ---------------------------------------------------------------------------
# Высокоуровневая функция: построение аналитического вида
# ---------------------------------------------------------------------------

def build_analytic_views_for_front_top_side(
    shape: TopoDS_Shape,
    edge_samples: int = 32,
    grid_size: int = 256,
    deflection: float = 5e-4,
    angle: float = 0.5,
) -> Dict[str, AnalyticView2D]:
    """
    1) Строит Ax2 для front/top/side на основе OBB-осей.
    2) Для каждого вида считает z-buffer видимости граней.
    3) На основе видимости граней строит AnalyticView2D (вершины/рёбра/грани)
       с заполненными типами поверхностей и типами линий.
    """
    view_axes_map = make_default_front_top_side_ax2(shape)
    analytic_views: Dict[str, AnalyticView2D] = {}

    for name, ax2 in view_axes_map.items():
        face_vis = compute_face_visibility_by_zbuffer(
            shape,
            ax2,
            grid_size=grid_size,
            deflection=deflection,
            angle=angle,
        )

        view = build_analytic_view_2d(
            shape,
            view_ax2=ax2,
            view_name=name,
            face_visibility=face_vis,
            n_edge_samples=edge_samples,
        )
        analytic_views[name] = view

    return analytic_views


# ---------------------------------------------------------------------------
# Примеры использования связей vertex–edge–face
# ---------------------------------------------------------------------------

def get_face_edges_and_vertices_on_view(view: AnalyticView2D, face_index: int) -> Tuple[List, Set]:
    """
    Возвращает:
      - список рёбер (DraftEdge2D), принадлежащих данной грани,
      - множество индексов вершин, участвующих в этих рёбрах.
    # пример:
    face_idx = 10
    edges_on_face, vertices_on_face = get_face_edges_and_vertices_on_view(front_view, face_idx)
    print(f"Грань #{face_idx}: рёбер={len(edges_on_face)}, вершин={len(vertices_on_face)}")
    for e in edges_on_face[:5]:
        print(f"  edge {e.edge_index}, faces={e.face_indices}, curve_type={e.curve_type}, vis={e.visibility}")
    """
    edges_on_face = []
    vertex_indices: set[int] = set()

    for e in view.edges.values():
        if face_index in e.face_indices:
            edges_on_face.append(e)
            if e.v_start is not None:
                vertex_indices.add(e.v_start)
            if e.v_end is not None:
                vertex_indices.add(e.v_end)

    vertices_on_face = [view.vertices[v_idx] for v_idx in vertex_indices]
    return edges_on_face, vertices_on_face

def describe_edge(view: AnalyticView2D, edge_index: int):
    """
    Для заданного ребра — к каким граням оно принадлежит и видимость.
    """
    e = view.edges[edge_index]

    print(f"Ребро #{edge_index}:")
    print(f"  тип кривой: {e.curve_type}")
    print(f"  видимость:  {e.visibility}")
    print(f"  граней:     {e.face_indices}")
    print(f"  вершины:    {e.v_start} -> {e.v_end}")
    print(f"  длина полилинии: {len(e.points)} точек")

    # Вершины как объекты
    v_start = view.vertices.get(e.v_start)
    v_end   = view.vertices.get(e.v_end)

    if v_start:
        print(f"    стартовая вершина: id={v_start.vertex_index}, visible={v_start.visible}")
    if v_end:
        print(f"    конечная вершина:  id={v_end.vertex_index}, visible={v_end.visible}")

def describe_vertex(view: AnalyticView2D, vertex_index: int):
    """
    Для заданной вершины — какие рёбра и какие грани вокруг неё
    """
    v = view.vertices[vertex_index]
    print(f"Вершина #{vertex_index}: (x={v.x:.3f}, y={v.y:.3f}), visible={v.visible}")
    print(f"  инцидентные рёбра: {v.incident_edges}")

    face_indices: Set[int] = set()
    for e_idx in v.incident_edges:
        e = view.edges[e_idx]
        face_indices.update(e.face_indices)

    print(f"  прилегающие грани: {sorted(face_indices)}")

def get_visible_elements(view: AnalyticView2D) -> Dict[str, List]:
    """
    Все видимые рёбра и вершины на виде
    """
    visible_edges = [e for e in view.edges.values() 
                     if e.visibility in ("visible", "silhouette")]

    visible_vertices = [v for v in view.vertices.values() if v.visible]

    return {
        'edges': visible_edges,
        'vertices': visible_vertices
    }

def get_visible_cylindrical_edges(view: AnalyticView2D) -> List:
    """
    Все видимые цилиндрические поверхности и их ребра на фронтальном виде
    """
    cyl_faces = {idx for idx, f in view.faces.items()
                 if f.surface_type == "cylinder" and f.visible}

    cyl_edges = []
    for e in view.edges.values():
        if any(fi in cyl_faces for fi in e.face_indices) and e.visibility in ("visible", "silhouette"):
            cyl_edges.append(e)
    return cyl_edges

def describe_cylindrical_face_on_view(view: AnalyticView2D, face_index: int):
    """
    Для одного цилиндра — построить локальный набор рёбер/вершин на фронте
    """
    face_info = view.faces[face_index]
    if face_info.surface_type != "cylinder":
        print(f"Грань #{face_index} не цилиндрическая (тип: {face_info.surface_type})")
        return

    edges_on_face = []
    vertex_ids = set()

    for e in view.edges.values():
        if face_index in e.face_indices:
            edges_on_face.append(e)
            if e.v_start is not None:
                vertex_ids.add(e.v_start)
            if e.v_end is not None:
                vertex_ids.add(e.v_end)

    print(f"Цилиндрическая грань #{face_index}: видима={face_info.visible}, рёбер={len(edges_on_face)}, вершин={len(vertex_ids)}")

    for e in edges_on_face:
        print(f"  edge {e.edge_index}: curve={e.curve_type}, vis={e.visibility}, line={e.line_style}")

    vertices = [view.vertices[vid] for vid in vertex_ids]
    for v in vertices:
        print(f"  vertex {v.vertex_index}: ({v.x:.2f}, {v.y:.2f}), visible={v.visible}, edges={v.incident_edges}")

def find_silhouette_edges(view: AnalyticView2D) -> List:
    """
    “Силуэтные” рёбра (граница между видимым и невидимым)
    """
    result = []
    for e in view.edges.values():
        if len(e.face_indices) != 2:
            continue
        f1, f2 = e.face_indices
        vis1 = view.faces[f1].visible
        vis2 = view.faces[f2].visible
        if vis1 != vis2:
            result.append(e)
    return result
