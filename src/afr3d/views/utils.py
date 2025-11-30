from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line


def sample_hlr_edge_2d(edge, n_samples: int = 32):
    """
    Семплируем ребро из HLR-shape.

    ВАЖНО:
      HLRBRep_HLRToShape уже выдал рёбра в плоскости проекции,
      поэтому мы просто берём p.X(), p.Y().
    """
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

        # п3d уже лежит в плоскости вида
        x = p3d.X()
        y = p3d.Y()
        pts_2d.append((x, y))

    return pts_2d