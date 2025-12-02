import matplotlib.pyplot as plt

from pathlib import Path
from collections import Counter, defaultdict
from typing import Dict, List, Tuple, Optional, Set

from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Ax2
from OCC.Core.HLRAlgo import HLRAlgo_Projector
from OCC.Core.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE

from OCC.Core.GeomAbs import (
    GeomAbs_Line,
    GeomAbs_Circle,
    GeomAbs_Ellipse,
    GeomAbs_BSplineCurve,
    GeomAbs_BezierCurve,
    GeomAbs_OtherCurve,
)
from OCC.Core.TopoDS import (
    TopoDS_Shape,
    TopoDS_Face,
    TopoDS_Edge,
    TopoDS_Vertex,
    topods
)
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint

from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound

from afr3d.views.analytic import (
    AnalyticView2D, DraftEdge2D, DraftVertex2D, 
    DraftEdgeSegment2D,
    project_point_to_view_2d, build_analytic_view_2d
)
from afr3d.views.utils import sample_hlr_edge_2d
from afr3d.views.visibility import compute_face_visibility_by_zbuffer

COLOR_BY_TYPE = {
    "line": "black",
    "circle": "red",
    "ellipse": "green",
    "bspline": "blue",
    "bezier": "magenta",
    "other": "orange",
}

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


def _union_compounds(shapes):
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    has_any = False
    for shp in shapes:
        if shp is None:
            continue
        if shp.IsNull():
            continue
        builder.Add(comp, shp)
        has_any = True
    return comp if has_any else None


def build_hlr_projection_ax2(shape: TopoDS_Shape, view_ax2: gp_Ax2):
    algo = HLRBRep_Algo()
    algo.Add(shape)

    projector = HLRAlgo_Projector(view_ax2)
    algo.Projector(projector)
    algo.Update()
    algo.Hide()

    hlr_to_shape = HLRBRep_HLRToShape(algo)

    visible = hlr_to_shape.VCompound()
    hidden  = hlr_to_shape.HCompound()

    # гладкие ребра
    try:
        rg1_v = hlr_to_shape.Rg1LineVCompound()
    except Exception:
        rg1_v = None
    try:
        rg1_h = hlr_to_shape.Rg1LineHCompound()
    except Exception:
        rg1_h = None

    # C2 (опционально)
    try:
        rgn_v = hlr_to_shape.RgNLineVCompound()
    except Exception:
        rgn_v = None
    try:
        rgn_h = hlr_to_shape.RgNLineHCompound()
    except Exception:
        rgn_h = None

    # контуры
    try:
        out_v = hlr_to_shape.OutLineVCompound()
    except Exception:
        out_v = None
    try:
        out_h = hlr_to_shape.OutLineHCompound()
    except Exception:
        out_h = None

    outline_visible = _union_compounds([rg1_v, rgn_v, out_v])
    outline_hidden  = _union_compounds([rg1_h, rgn_h, out_h])

    projection = {
        "visible":         visible,
        "hidden":          hidden,
        "outline_visible": outline_visible,
        "outline_hidden":  outline_hidden,
    }
    return projection, view_ax2


def sample_face_edge_2d(edge, ax2: gp_Ax2, n_samples: int = 32):
    pts_2d = []
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


# def draw_hlr_compound_by_type(ax, compound, This function modified in debug.py
#                               visible=True,
#                               is_outline=False):
#     """
#     Рисуем HLR-shape. НЕ используем ax2 — координаты уже в плоскости вида.
#     """
#     if compound is None or compound.IsNull():
#         return

#     exp = TopExp_Explorer(compound, TopAbs_EDGE)
#     legend_done = set()

#     while exp.More():
#         edge = exp.Current()
#         exp.Next()

#         try:
#             bac = BRepAdaptor_Curve(edge)
#             ctype = bac.GetType()
#         except Exception:
#             continue

#         tname = curve_type_name(ctype)
#         color = COLOR_BY_TYPE.get(tname, "gray")

#         pts_2d = sample_hlr_edge_2d(edge)
#         if len(pts_2d) < 2:
#             continue

#         xs = [p[0] for p in pts_2d]
#         ys = [p[1] for p in pts_2d]

#         linestyle = "-" if visible else "--"
#         alpha = 1.0 if visible else 0.4
#         linewidth = 2.0 if is_outline else 0.8

#         ax.plot(xs, ys,
#                 color=color,
#                 linestyle=linestyle,
#                 alpha=alpha,
#                 linewidth=linewidth)


def draw_analytic_view_edges(
    view: AnalyticView2D,
    *,
    show_visible: bool = True,
    show_silhouette: bool = True,
    show_hidden: bool = True,
    fig_ax=None,
    title: str | None = None,
):
    """
    Простейший рендер аналитического вида через matplotlib.

    Использует:
      - edge.visibility  ('visible', 'silhouette', 'hidden', 'unknown')
      - edge.line_style  ('solid', 'hidden', 'solid_thick', ...)
    """
    if fig_ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))
    else:
        fig, ax = fig_ax

    ax.set_aspect("equal", "box")

    for e in view.edges.values():
        if not e.points or len(e.points) < 2:
            continue

        # фильтрация по видимости
        if e.visibility == "visible" and not show_visible:
            continue
        if e.visibility == "silhouette" and not show_silhouette:
            continue
        if e.visibility == "hidden" and not show_hidden:
            continue

        xs = [p[0] for p in e.points]
        ys = [p[1] for p in e.points]

        # выбираем стиль по line_style
        if e.line_style == "solid":
            ls = "-"
            lw = 1.0
            alpha = 1.0
        elif e.line_style == "solid_thick":
            ls = "-"
            lw = 1.8
            alpha = 1.0
        elif e.line_style == "hidden":
            ls = "--"
            lw = 0.8
            alpha = 0.6
        else:
            ls = "-"
            lw = 0.8
            alpha = 0.7

        ax.plot(xs, ys, linestyle=ls, linewidth=lw, alpha=alpha, color="black")

    ax.grid(True)
    if title is None:
        title = f"Analytic view: {view.view_name}"
    ax.set_title(title)

    return fig, ax


def draw_hlr_compound(ax, compound, *, color="black", linestyle="-", linewidth=1.0, alpha=1.0):
    """
    Рисуем один compound (visible / hidden / outline) HLR на axes.
    """
    if compound is None or compound.IsNull():
        return

    exp = TopExp_Explorer(compound, TopAbs_EDGE)
    while exp.More():
        edge = exp.Current()
        exp.Next()

        pts = sample_hlr_edge_2d(edge)
        if len(pts) < 2:
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=color, linestyle=linestyle,
                linewidth=linewidth, alpha=alpha)


# ---------------------------------------------------------------------------
# FOR ANALYTIC VIEW
# ---------------------------------------------------------------------------


def draw_edge_segments(
    segments: List[DraftEdgeSegment2D],
    *,
    fig_ax=None,
    title: Optional[str] = None,
):
    """
    Рисуем список сегментов (после классификации через HLR).

    visibility / line_style используются так же, как для рёбер.
    """
    if fig_ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))
    else:
        fig, ax = fig_ax

    ax.set_aspect("equal", "box")

    for seg in segments:
        pts = seg.points
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        style = seg.line_style
        if style == "solid":
            ls = "-"
            lw = 1.0
            alpha = 1.0
        elif style == "solid_thick":
            ls = "-"
            lw = 1.8
            alpha = 1.0
        elif style == "hidden":
            ls = "--"
            lw = 0.8
            alpha = 0.6
        else:
            ls = "-"
            lw = 0.8
            alpha = 0.7

        ax.plot(xs, ys, linestyle=ls, linewidth=lw, alpha=alpha, color="black")

    ax.grid(True)
    if title is None:
        title = "Analytic segments (HLR-refined)"
    ax.set_title(title)
    return fig, ax


# ---------------------------------------------------------------------------
# Highlights functions
# ---------------------------------------------------------------------------

def highlight_face_on_view(ax, face: TopoDS_Face, view_ax2,
                           color="yellow", linewidth=3.0):
    """
    Обводим контур грани, проектируя исходную 3D-геометрию в координаты вида.
    Именно так мы приводим faces к тому же 2D, что использует HLR.
    """
    exp = TopExp_Explorer(face, TopAbs_EDGE)
    while exp.More():
        edge = exp.Current()
        exp.Next()

        pts = sample_face_edge_2d(edge, view_ax2)
        if len(pts) < 2:
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys,
                color=color,
                linewidth=linewidth,
                zorder=10)


def highlight_edge_on_view(ax,
                           proj_edges: Dict[int, DraftEdge2D],
                           edge_index: int,
                           color: str = "cyan",
                           linewidth: float = 2.0):
    """
    Подсветить одно ребро по индексу edge_index (индекс из edge_map).
    """
    pe = proj_edges.get(edge_index)
    if pe is None:
        print(f"[highlight_edge_on_view] нет ребра с индексом {edge_index}")
        return

    pts = pe.points
    if len(pts) < 2:
        return

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, linewidth=linewidth, zorder=15)


def highlight_vertex_on_view(ax,
                             proj_vertices: Dict[int, DraftVertex2D],
                             vertex_index: int,
                             color: str = "yellow",
                             size: float = 40.0):
    """
    Подсветить вершину по индексу vertex_index (индекс из vertex_map).
    """
    pv = proj_vertices.get(vertex_index)
    if pv is None:
        print(f"[highlight_vertex_on_view] нет вершины с индексом {vertex_index}")
        return

    ax.scatter([pv.x], [pv.y],
               s=size,
               color=color,
               edgecolors="black",
               linewidths=0.5,
               zorder=20)
    

# ---------------------------------------------------------------------------
# High-level function
# ---------------------------------------------------------------------------

def highlight_elements_on_view(
        shape,
        view_ax2,
        edge_idx: int,
        vertex_idx: int,
        view_name='view_name'
):
    projection, view_ax2 = build_hlr_projection_ax2(shape, view_ax2)
    face_vis = compute_face_visibility_by_zbuffer(shape, view_ax2)
    analytic_view = build_analytic_view_2d(
        shape,
        view_ax2=view_ax2,
        view_name=view_name,
        face_visibility=face_vis,
    )

    visible         = projection["visible"]
    hidden          = projection["hidden"]
    outline_visible = projection["outline_visible"]
    outline_hidden  = projection["outline_hidden"]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal", "box")

    draw_hlr_compound_by_type(ax, outline_hidden,  visible=False, is_outline=True)
    draw_hlr_compound_by_type(ax, hidden,          visible=False, is_outline=False)
    draw_hlr_compound_by_type(ax, outline_visible, visible=True,  is_outline=True)
    draw_hlr_compound_by_type(ax, visible,         visible=True,  is_outline=False)

    # подсветим ребро и вершину по индексам из аналитического вида
    e_idx = edge_idx
    v_idx = vertex_idx

    highlight_edge_on_view(ax, analytic_view.edges, e_idx, color="cyan", linewidth=2.5)
    highlight_vertex_on_view(ax, analytic_view.vertices, v_idx, color="yellow", size=50.0)

    ax.grid(True)
    ax.set_title(f"HLR + analytic edge/vertex highlight, view_name={view_name}")
    plt.show()