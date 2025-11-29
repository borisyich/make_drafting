# afr3d/drafting/model.py

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional, Literal

from OCC.Core.gp import gp_Ax2, gp_Dir


VIEW_ROTATION = {
    "+d1": 0,      # фронт
    "-d1": 180,    # задний
    "+d2": 90,     # сверху
    "-d2": -90,    # снизу
    "+d3": 90,     # боковой
    "-d3": -90,
}

HoleKind = Literal["through", "blind", "unknown"]
HoleGeometryType = Literal["circular_simple", "circular_stepped", "non_circular", "unknown"]

Point2D = Tuple[float, float]

@dataclass
class DraftCurveKind(Enum):
    LINE = auto()
    CIRCLE = auto()
    ARC = auto()
    POLYLINE = auto()
    # ELLIPSE и прочее можно добавить позже


@dataclass
class CoverageKind(Enum):
    FULL = auto()     # полностью покрыто HLR (доверяем)
    PARTIAL = auto()  # HLR покрыл только часть (есть "дырки")
    NONE = auto()     # HLR вообще не дал покрытия

@dataclass
class DraftVertex2D:
    id: int
    x: float
    y: float

    # связь с 3D
    source_vertex_index: Optional[int] = None  # номер в списке TopoDS_Vertex
    # можно позже хранить hash/Guid исходного TopoDS_Vertex

@dataclass
class DraftEdge2D:
    id: int
    v_start: int         # id вершины
    v_end: int           # id вершины
    kind: DraftCurveKind = DraftCurveKind.LINE
    visible: bool = False

    # если это кусок окружности/дуги:
    center: Optional[Point2D] = None
    radius: Optional[float] = None
    start_angle: Optional[float] = None
    end_angle: Optional[float] = None

    # связь с 3D / AFR
    source_edge_index: Optional[int] = None  # индекс ребра в TopTools_IndexedMapOfShape или свой индекс
    source_curve_id: Optional[int] = None
    feature_id: Optional[str] = None     # например "hole:3"
    layer: str = ""                      # "outline", "hidden", "center", ...

    # NEW: к каким граням 3D-тела относится это 2D-ребро
    face_indices: List[int] = field(default_factory=list)

    # покрытие HLR
    coverage: CoverageKind = CoverageKind.FULL
    coverage_gap: float = 0.0  # длина "дыр" по этому ребру (в мм)

# Кривые более высокого уровня (после “склейки” рёбер)
# Отдельно от “сырых” рёбер удобно хранить уже объединённые сущности:
@dataclass
class DraftCurve2D:
    kind: DraftCurveKind
    visible: bool = True

    # LINE / POLYLINE:
    points: List[Point2D] = field(default_factory=list)

    # CIRCLE / ARC:
    center: Optional[Point2D] = None
    radius: Optional[float] = None
    start_angle: Optional[float] = None
    end_angle: Optional[float] = None

    # связь со старыми рёбрами:
    edge_ids: List[int] = field(default_factory=list)
    feature_id: Optional[str] = None
    layer: str = ""

    # покрытие HLR (для окружностей/дуг)
    coverage: CoverageKind = CoverageKind.FULL
    coverage_gap: float = 0.0   # суммарная длина по дуге/кругу, не покрытая HLR

# новый аналитический вид
@dataclass
class DraftView2D:
    name: str                     # "front", "top", "side", "+d1", "-d1", ...
    ax2: gp_Ax2                   # базис проекции

    # "сырые" вершины и рёбра (из топологии или HLR):
    vertices: List[DraftVertex2D] = field(default_factory=list)
    edges: List[DraftEdge2D] = field(default_factory=list)

    # более высокоуровневые кривые после аналитики:
    curves: List[DraftCurve2D] = field(default_factory=list)

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
    ax2: gp_Ax2       # базис проекции (используется в HLR)


@dataclass
class ViewSet:
    front: OrthoViewDef
    top: OrthoViewDef
    side: OrthoViewDef


@dataclass
class ProjectedSegment2D:
    x1: float
    y1: float
    x2: float
    y2: float
    visible: bool

@dataclass
class HoleSegment:
    kind: Literal["cyl", "cone"]
    radius: float | None
    length: float
    face_indices: List[int]

@dataclass
class ChamferInfo:
    face_index: int
    length: float
    semi_angle_deg: float
    side: str

@dataclass
class HoleAFR:
    id: int
    geometry_type: HoleGeometryType
    kind: HoleKind
    axis_origin: Tuple[float, float, float] | None
    axis_dir: Tuple[float, float, float] | None
    nominal_radius: float | None
    segments: List[HoleSegment]
    side_face_indices: List[int]
    opening_faces: List[int]
    bottom_faces: List[int]
    chamfers: List[ChamferInfo]

    name: Optional[str] = None
    feature_id: Optional[str] = None
