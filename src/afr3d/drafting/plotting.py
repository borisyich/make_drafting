import math
import matplotlib.pyplot as plt
from typing import Optional, Dict

from afr3d.drafting.model import DraftView2D, DraftVertex2D, DraftCurveKind, VIEW_ROTATION


def rotate_point_deg(x: float, y: float, angle_deg: float) -> tuple[float, float]:
    if abs(angle_deg) < 1e-9:
        return x, y
    if abs(angle_deg - 90) < 1e-9:
        return -y, x
    if abs(angle_deg + 90) < 1e-9:
        return y, -x
    if abs(angle_deg - 180) < 1e-9 or abs(angle_deg + 180) < 1e-9:
        return -x, -y

    rad = math.radians(angle_deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return c * x - s * y, s * x + c * y

def plot_analytic_view_with_vertices(
    ax,
    view: DraftView2D,
    title: Optional[str] = None,
    line_width: float = 0.5,
    show_hidden: bool = False,
):
    """
    Аналитический вид:
      - линейные рёбра (DraftEdge2D) с учётом visible;
      - окружности и дуги (DraftCurve2D: CIRCLE/ARC/POLYLINE);
      - вершины — звёздочками.
    """

    angle = VIEW_ROTATION.get(view.name, 0)
    vertices_by_id: Dict[int, DraftVertex2D] = {v.id: v for v in view.vertices}

    # --- ЛИНЕЙНЫЕ РЁБРА ---
    for e in view.edges:
        is_visible = getattr(e, "visible", True)

        if not is_visible and not show_hidden:
            continue

        v_start = vertices_by_id.get(getattr(e, "v_start", None))
        v_end = vertices_by_id.get(getattr(e, "v_end", None))
        if v_start is None or v_end is None:
            continue

        x1, y1 = rotate_point_deg(v_start.x, v_start.y, angle)
        x2, y2 = rotate_point_deg(v_end.x, v_end.y, angle)

        linestyle = "-" if is_visible else "--"

        ax.plot(
            [x1, x2],
            [y1, y2],
            linewidth=line_width,
            color="black",
            linestyle=linestyle,
        )

    # --- КРИВЫЕ (окружности, дуги, полилинии) ---
    for c in view.curves:
        kind = getattr(c, "kind", None)
        is_visible = getattr(c, "visible", True)

        if not is_visible and not show_hidden:
            continue

        linestyle = "-" if is_visible else "--"

        if kind is DraftCurveKind.CIRCLE:
            if c.center is None or c.radius is None:
                continue
            cx, cy = c.center
            r = c.radius
            cxr, cyr = rotate_point_deg(cx, cy, angle)

            circle = plt.Circle(
                (cxr, cyr),
                r,
                fill=False,
                linewidth=line_width,
                linestyle=linestyle,
                color="black",
            )
            ax.add_patch(circle)

        elif kind is DraftCurveKind.ARC:
            if c.center is None or c.radius is None:
                continue
            cx, cy = c.center
            r = c.radius
            a0 = c.start_angle if c.start_angle is not None else 0.0
            a1 = c.end_angle if c.end_angle is not None else 0.0

            # дискретизируем дугу в полилинию
            n = 32
            pts = []
            for i in range(n + 1):
                t = a0 + (a1 - a0) * i / n
                x = cx + r * math.cos(t)
                y = cy + r * math.sin(t)
                xr, yr = rotate_point_deg(x, y, angle)
                pts.append((xr, yr))

            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(
                xs,
                ys,
                linewidth=line_width,
                color="black",
                linestyle=linestyle,
            )

        elif kind is DraftCurveKind.POLYLINE:
            if not c.points:
                continue
            pts = [rotate_point_deg(px, py, angle) for (px, py) in c.points]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(
                xs,
                ys,
                linewidth=line_width,
                color="black",
                linestyle=linestyle,
            )

        # остальные типы можно добавить позже

    # --- ВЕРШИНЫ ---
    if view.vertices:
        v_rot = [rotate_point_deg(v.x, v.y, angle) for v in view.vertices]
        xs = [vx for vx, _ in v_rot]
        ys = [vy for _, vy in v_rot]

        ax.scatter(xs, ys, marker="*", s=20, color="black")

    ax.set_aspect("equal", adjustable="box")
    ax.invert_yaxis()
    if title:
        ax.set_title(title)