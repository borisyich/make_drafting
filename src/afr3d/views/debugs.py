import math
import matplotlib.pyplot as plt

from collections import Counter, defaultdict
from typing import Dict, List, Tuple, Optional, Set, Union, Any
from OCC.Core.TopoDS import TopoDS_Shape

from afr3d.views.analytic import (
    AnalyticView2D, DraftEdgeSegment2D, _collect_hlr_samples
)
from afr3d.views.plotting import draw_hlr_compound


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def draw_hlr_compound_by_type(
    projection: Dict[str, TopoDS_Shape],
    ax,
    *,
    colors: Optional[Dict[str, str]] = None,
    linewidth: float = 1.0,
    alpha: float = 1.0,
    outline_scale: float = 1.5,
    hidden_linestyle: str = "--",
):
    """
    Рисуем все HLR-компаунды (visible / hidden / outline_visible / outline_hidden)
    на один Axes.

    projection:
        dict вида:
            {
              "visible":         TopoDS_Shape,
              "hidden":          TopoDS_Shape,
              "outline_visible": TopoDS_Shape или None,
              "outline_hidden":  TopoDS_Shape или None,
            }

    ax:
        matplotlib Axes.

    colors:
        dict с цветами по ключам projection:
            {
              "visible":         "#000000",
              "hidden":          "#888888",
              "outline_visible": "#0000ff",
              "outline_hidden":  "#8888ff",
            }
        Если None — используются значения по умолчанию.

    linewidth:
        базовая толщина линии для обычных (visible/hidden) линий.

    alpha:
        базовая прозрачность для всех линий (можно переиграть при желании).

    outline_scale:
        во сколько раз outline-линии толще обычных.

    hidden_linestyle:
        стиль линии для скрытых рёбер ("--", "-.", ":" и т.п.).
    """
    if colors is None:
        colors = {
            "visible":         "black",
            "hidden":          "0.5",      # серый
            "outline_visible": "blue",
            "outline_hidden":  "0.6",      # светло-серый / голубоватый
        }

    # Видимые внутренние рёбра
    vis = projection.get("visible")
    if vis is not None:
        draw_hlr_compound(
            ax,
            vis,
            color=colors.get("visible", "black"),
            linestyle="-",
            linewidth=linewidth,
            alpha=alpha,
        )

    # Скрытые внутренние рёбра
    hid = projection.get("hidden")
    if hid is not None:
        draw_hlr_compound(
            ax,
            hid,
            color=colors.get("hidden", "0.5"),
            linestyle=hidden_linestyle,
            linewidth=linewidth * 0.9,
            alpha=alpha * 0.8,
        )

    # Видимые контуры / силуэты
    out_v = projection.get("outline_visible")
    if out_v is not None:
        draw_hlr_compound(
            ax,
            out_v,
            color=colors.get("outline_visible", "blue"),
            linestyle="-",
            linewidth=linewidth * outline_scale,
            alpha=alpha,
        )

    # Скрытые контуры / силуэты
    out_h = projection.get("outline_hidden")
    if out_h is not None:
        draw_hlr_compound(
            ax,
            out_h,
            color=colors.get("outline_hidden", "0.6"),
            linestyle=hidden_linestyle,
            linewidth=linewidth * outline_scale * 0.9,
            alpha=alpha * 0.8,
        )


def debug_plot_edge_and_segments(
    view: AnalyticView2D,
    edge_indices: Union[int, List[int]],
    segments: List[DraftEdgeSegment2D],
    projection: Optional[Dict[str, TopoDS_Shape]] = None,
    *,
    show_hlr=True,
):
    """
    Рисует для одного или нескольких рёбер:
      - исходные аналитические рёбра (серым),
      - сегменты этих рёбер (цвет/стиль по visibility),
      - опционально HLR сверху (тонкими линиями).

    edge_indices: int или [int, int, ...]
    """
    # Приводим индексы к списку
    if isinstance(edge_indices, int):
        edge_indices = [edge_indices]

    # Готовим фигуру
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", "box")

    # --- 1. Рисуем исходные рёбра ---
    for e_idx in edge_indices:
        e = view.edges.get(e_idx)
        if e is None or len(e.points) < 2:
            print(f"[DEBUG] Edge {e_idx} not found or too short.")
            continue

        xs = [p[0] for p in e.points]
        ys = [p[1] for p in e.points]

        ax.plot(xs, ys, linestyle="-", color="0.7", linewidth=5.0,
                label=f"edge {e_idx} original")

    # --- 2. Рисуем сегменты ---
    for seg in segments:
        if seg.parent_edge_index not in edge_indices:
            continue

        pts = seg.points
        if len(pts) < 2:
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        # стиль по видимости
        if seg.visibility == "visible":
            ls, lw, alpha = "-", 1.5, 1.0
        elif seg.visibility == "hidden":
            ls, lw, alpha = "--", 1.0, 0.8
        elif seg.visibility == "silhouette":
            ls, lw, alpha = "-", 2.0, 1.0
        else:
            ls, lw, alpha = "-", 1.0, 0.5

        ax.plot(xs, ys, linestyle=ls, linewidth=lw, alpha=alpha, color="black")

    # --- 3. HLR сверху (опционально) ---
    if show_hlr and projection is not None:
        draw_hlr_compound_by_type(
            projection,            # <-- 1-й аргумент (positional)
            ax,                    # <-- 2-й аргумент (positional!)
            colors={
                "visible": "#009900",
                "hidden": "#bb0000",
                "outline_visible": "#0000bb",
                "outline_hidden": "#8888ff",
            },
            linewidth=0.5,
            alpha=0.4,
        )

    # --- финальный вид ---
    ax.set_title(f"Debug edges: {edge_indices}")
    ax.grid(True)
    ax.legend()

    return fig, ax


def debug_plot_single_segment(
    view: AnalyticView2D,
    seg: DraftEdgeSegment2D,
    projection: Optional[Dict[str, TopoDS_Shape]] = None,
):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", "box")

    # исходное ребро серым
    e = view.edges.get(seg.parent_edge_index)
    if e is not None and len(e.points) >= 2:
        xs = [p[0] for p in e.points]
        ys = [p[1] for p in e.points]
        ax.plot(xs, ys, "-", color="0.8", linewidth=1.0, label="edge")

    # сам сегмент жирно
    pts = seg.points
    sx = [p[0] for p in pts]
    sy = [p[1] for p in pts]

    if seg.visibility == "visible":
        ls, lw = "-", 2.0
    elif seg.visibility == "hidden":
        ls, lw = "--", 1.5
    elif seg.visibility == "silhouette":
        ls, lw = "-", 2.5
    else:
        ls, lw = "-", 1.5

    ax.plot(sx, sy, ls, color="black", linewidth=lw, label=f"seg ({seg.visibility})")

    # HLR сверху (опционально)
    if projection is not None:
        draw_hlr_compound_by_type(
            projection,
            ax,
            linewidth=0.5,
            alpha=0.5,
        )

    ax.grid(True)
    ax.legend()
    return fig, ax


# ---------------------------------------------------------------------------
# Analytic
# ---------------------------------------------------------------------------


def _polyline_length(pts):
    if len(pts) < 2:
        return 0.0
    acc = 0.0
    for (x1, y1), (x2, y2) in zip(pts[:-1], pts[1:]):
        dx = x2 - x1
        dy = y2 - y1
        acc += math.hypot(dx, dy)
    return acc


def debug_edge_segment_coverage(
    view: AnalyticView2D,
    segments: List[DraftEdgeSegment2D],
    *,
    length_tol_abs: float = 1e-6,
    length_tol_rel: float = 0.1,
) -> Dict[int, Dict[str, float]]:
    """
    Диагностирует, какие рёбра "потеряли" длину при переходе к сегментам.

    Возвращает словарь:
      edge_index -> {
          'edge_len': L_edge,
          'seg_len':  L_segments_sum,
          'coverage': seg_len / (edge_len + eps),
      }

    И печатает подозрительные рёбра:
      - coverage ≈ 0 (сегментов нет вообще)
      - coverage << 1 (например, < (1 - length_tol_rel))
    """
    # 1. суммарная длина сегментов по родительскому ребру
    seg_len_by_edge: Dict[int, float] = defaultdict(float)
    for seg in segments:
        if seg.parent_edge_index is None or seg.parent_edge_index < 0:
            # HLR-only сегменты, их игнорируем здесь
            continue
        seg_len_by_edge[seg.parent_edge_index] += _polyline_length(seg.points)

    # 2. длина исходных рёбер
    coverage_info: Dict[int, Dict[str, float]] = {}
    print("[DEBUG] Edge coverage by segments:")

    for e_idx, e in view.edges.items():
        L_edge = _polyline_length(e.points)
        L_seg = seg_len_by_edge.get(e_idx, 0.0)
        if L_edge <= length_tol_abs:
            coverage = 1.0
        else:
            coverage = L_seg / (L_edge + 1e-12)

        coverage_info[e_idx] = {
            "edge_len": L_edge,
            "seg_len": L_seg,
            "coverage": coverage,
        }

        # печатаем подозрительные случаи
        if L_edge > length_tol_abs:
            if L_seg <= length_tol_abs:
                print(
                    f"  Edge {e_idx:4d}: LOST ENTIRELY, "
                    f"L_edge={L_edge:.4g}, L_seg={L_seg:.4g}, "
                    f"vis={getattr(e, 'visibility', None)}"
                )
            elif coverage < (1.0 - length_tol_rel):
                print(
                    f"  Edge {e_idx:4d}: PARTIAL LOSS, "
                    f"coverage={coverage:.3f}, "
                    f"L_edge={L_edge:.4g}, L_seg={L_seg:.4g}, "
                    f"vis={getattr(e, 'visibility', None)}"
                )

    return coverage_info


def snapshot_edge_visibility(view: AnalyticView2D) -> Dict[int, str]:
    """
    Делает снимок текущей видимости рёбер (до переразметки).
    """
    return {idx: e.visibility for idx, e in view.edges.items()}


def debug_edge_visibility_diff(
    view: AnalyticView2D,
    old_visibility: Dict[int, str],
    segments: List[DraftEdgeSegment2D],
):
    """
    Сравнивает старую видимость рёбер (old_visibility)
    с новой, выведенной по сегментам.

    Новая видимость по ребру считается так:
      - если все сегменты visible  -> 'visible'
      - если все hidden            -> 'hidden'
      - если смесь                 -> 'silhouette'
      - если сегментов нет         -> 'lost'
    """
    # 1. группируем сегменты по родительскому ребру
    seg_by_edge: Dict[int, List[DraftEdgeSegment2D]] = defaultdict(list)
    for seg in segments:
        if seg.parent_edge_index is None or seg.parent_edge_index < 0:
            continue
        seg_by_edge[seg.parent_edge_index].append(seg)

    print("[DEBUG] Edge visibility diff (old vs segments-derived):")

    for e_idx, e in view.edges.items():
        old_vis = old_visibility.get(e_idx, "unknown")
        segs = seg_by_edge.get(e_idx, [])

        if not segs:
            new_vis = "lost"
        else:
            vis_states = {s.visibility for s in segs}
            if vis_states == {"visible"}:
                new_vis = "visible"
            elif vis_states == {"hidden"}:
                new_vis = "hidden"
            elif vis_states <= {"visible", "hidden", "silhouette"}:
                if "silhouette" in vis_states or ( "visible" in vis_states and "hidden" in vis_states ):
                    new_vis = "silhouette"
                else:
                    # теоретически не должны сюда попасть
                    new_vis = ",".join(sorted(vis_states))
            else:
                new_vis = ",".join(sorted(vis_states))

        if new_vis != old_vis:
            L_edge = _polyline_length(e.points)
            print(
                f"  Edge {e_idx:4d}: "
                f"old={old_vis:10s} -> new={new_vis:10s} "
                f"(len={L_edge:.4g})"
            )


def debug_segment_vs_hlr_confusion(
    segments: List[DraftEdgeSegment2D],
    projection: Dict[str, TopoDS_Shape],
    *,
    n_samples_hlr: int = 64,
    tol_factor: float = 1e-3,
):
    """
    Строит confusion matrix между видимостью наших сегментов и HLR.

    Для каждого сегмента:
      - берём его точки,
      - для каждой точки ищем ближайший HLR-семпл (vis/hid/outline),
      - по большинству для этого сегмента определяем HLR-метку,
      - сравниваем с seg.visibility (visible/hidden/silhouette),
      - накапливаем статистику (по количеству сегментов и по длине).
    """
    hlr_pts = _collect_hlr_samples(projection, n_samples=n_samples_hlr)
    if not hlr_pts:
        print("[SEG HLR CONF] HLR samples are empty — nothing to compare.")
        return

    # Базовый размер сцены (для tol)
    xs = [p[0] for p in hlr_pts]
    ys = [p[1] for p in hlr_pts]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    diag = math.hypot(x_max - x_min, y_max - y_min) or 1.0
    tol2 = (tol_factor * diag) ** 2

    # Маппинг HLR-лейблов в "грубую" видимость
    def hlr_label_to_vis(lbl: str) -> str:
        if lbl in ("vis", "vis_outline"):
            return "visible"
        if lbl in ("hid", "hid_outline"):
            return "hidden"
        return "unknown"

    # Конфьюжн по количеству сегментов и по длине
    count_conf = Counter()
    length_conf = Counter()

    for seg in segments:
        pts = seg.points
        if len(pts) < 2:
            continue

        # длина сегмента (для веса)
        seg_len = _polyline_length(pts)

        # если длина совсем нулевая — пропускаем
        if seg_len <= 1e-9:
            continue

        # Для каждой точки — ближайший HLR-класс
        hlr_labels = []
        for (x, y) in pts:
            best_lbl = None
            best_d2 = tol2  # дальше этого расстояния не считаем совпадением

            for (hx, hy, hlabel) in hlr_pts:
                dx = x - hx
                dy = y - hy
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_lbl = hlabel

            if best_lbl is None:
                continue
            hlr_labels.append(best_lbl)

        if not hlr_labels:
            hlr_vis = "unknown"
        else:
            # мажоритарный HLR-класс -> грубая видимость
            most_hlr_lbl, _ = Counter(hlr_labels).most_common(1)[0]
            hlr_vis = hlr_label_to_vis(most_hlr_lbl)

        my_vis = seg.visibility or "unknown"

        # накапливаем
        count_conf[(hlr_vis, my_vis)] += 1
        length_conf[(hlr_vis, my_vis)] += seg_len

    # Печать результатов
    print("[SEG HLR CONF] confusion by segment count:")
    for (hlr_vis, my_vis), cnt in sorted(count_conf.items()):
        print(f"  HLR={hlr_vis:10s}  OUR={my_vis:10s} : {cnt:4d} segments")

    print("[SEG HLR CONF] confusion by total length:")
    for (hlr_vis, my_vis), total_len in sorted(length_conf.items()):
        print(f"  HLR={hlr_vis:10s}  OUR={my_vis:10s} : {total_len:8.3f} length")
    


def find_suspicious_hidden_segments(
    segments: List[DraftEdgeSegment2D],
    projection: Dict[str, TopoDS_Shape],
    *,
    n_samples_hlr: int = 64,
    tol_factor: float = 1e-3,
    min_visible_frac: float = 0.1,
) -> List[Dict[str, Any]]:
    """
    Ищем сегменты с visibility='hidden', у которых
    заметная доля точек по HLR на самом деле попадает
    на видимые линии.

    min_visible_frac:
        минимальная доля "видимых" HLR-точек (0..1),
        чтобы считать сегмент подозрительным.
    """
    hlr_pts = _collect_hlr_samples(projection, n_samples=n_samples_hlr)
    suspicious: List[Dict[str, Any]] = []

    if not hlr_pts:
        print("[SUSP HIDDEN] HLR samples empty.")
        return suspicious

    # масштаб
    xs = [p[0] for p in hlr_pts]
    ys = [p[1] for p in hlr_pts]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    diag = math.hypot(x_max - x_min, y_max - y_min) or 1.0
    tol2 = (tol_factor * diag) ** 2

    def is_visible_hlr(lbl: str) -> bool:
        return lbl in ("vis", "vis_outline")

    for seg_idx, seg in enumerate(segments):
        if seg.visibility != "hidden":
            continue

        pts = seg.points
        if len(pts) < 2:
            continue

        seg_len = _polyline_length(pts)
        if seg_len <= 1e-9:
            continue

        n_total = 0
        n_vis = 0

        for (x, y) in pts:
            best_lbl = None
            best_d2 = tol2
            for (hx, hy, hlabel) in hlr_pts:
                dx = x - hx
                dy = y - hy
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_lbl = hlabel

            if best_lbl is None:
                continue

            n_total += 1
            if is_visible_hlr(best_lbl):
                n_vis += 1

        if n_total == 0:
            continue

        frac_vis = n_vis / n_total

        if frac_vis >= min_visible_frac:
            suspicious.append(
                {
                    "seg_idx": seg_idx,
                    "parent_edge_index": seg.parent_edge_index,
                    "seg_len": seg_len,
                    "visible_frac": frac_vis,
                }
            )

    suspicious.sort(key=lambda d: d["visible_frac"], reverse=True)

    print(f"[SUSP HIDDEN] total suspicious segments: {len(suspicious)}")
    for m in suspicious[:20]:
        print(
            f"  seg_idx={m['seg_idx']:4d}, edge={m['parent_edge_index']:4d}, "
            f"len={m['seg_len']:7.3f}, visible_frac={m['visible_frac']:.3f}"
        )

    return suspicious
