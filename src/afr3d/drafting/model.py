# afr3d/drafting/model.py

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional

from OCC.Core.gp import gp_Ax2


Point2D = Tuple[float, float]

@dataclass
class DraftCurveKind(Enum):
    LINE = auto()
    CIRCLE = auto()
    ARC = auto()
    POLYLINE = auto()
    # ELLIPSE и прочее можно добавить позже

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
    visible: bool = True

    # если это кусок окружности/дуги:
    center: Optional[Point2D] = None
    radius: Optional[float] = None
    start_angle: Optional[float] = None
    end_angle: Optional[float] = None

    # связь с 3D / AFR
    source_edge_index: Optional[int] = None
    source_curve_id: Optional[int] = None
    feature_id: Optional[str] = None     # например "hole:3"
    layer: str = ""                      # "outline", "hidden", "center", ...

    # NEW: к каким граням 3D-тела относится это 2D-ребро
    face_indices: List[int] = field(default_factory=list)


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
