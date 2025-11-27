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
    angle = VIEW_ROTATION.get(view.name, 0)
    vertices_by_id: Dict[int, DraftVertex2D] = {v.id: v for v in view.vertices}

    for e in view.edges:
        print(getattr(e, "visible"))
        is_visible = getattr(e, "visible", True)

        if not is_visible and not show_hidden:
            continue  # скрытые просто не рисуем

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

    # вершины — как было
    if view.vertices:
        v_rot = [rotate_point_deg(v.x, v.y, angle) for v in view.vertices]
        xs = [vx for vx, _ in v_rot]
        ys = [vy for _, vy in v_rot]
        ax.scatter(xs, ys, marker="*", s=20, color="black")

    ax.set_aspect("equal", adjustable="box")
    ax.invert_yaxis()
    if title:
        ax.set_title(title)