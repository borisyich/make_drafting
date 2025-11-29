"""STEP file import utilities."""

from OCC.Core.BRep import BRep_Builder
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape, topods
from OCC.Core.TopExp import TopExp_Explorer


def extract_solids(root_shape: TopoDS_Shape) -> TopoDS_Compound:
    """
    Возвращает compound, содержащий только SOLID'ы из исходного shape.
    Это отсекает текст, разметку, вспомогательные тела и т.п.
    """
    if root_shape is None or root_shape.IsNull():
        return root_shape

    exp = TopExp_Explorer(root_shape, TopAbs_SOLID)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)

    has_solids = False
    while exp.More():
        solid = topods.Solid(exp.Current())
        builder.Add(comp, solid)
        has_solids = True
        exp.Next()

    return comp if has_solids else root_shape

def load_step(path) -> TopoDS_Shape:
    """Load a STEP file and return a unified ``TopoDS_Shape``.

    Parameters
    ----------
    path:
        Path to the STEP file on disk.
    """
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    return shape