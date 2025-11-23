"""STEP file import utilities."""

from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopoDS import TopoDS_Shape


def load_step(shape_path: str) -> TopoDS_Shape:
    """Load a STEP file and return a unified ``TopoDS_Shape``.

    Parameters
    ----------
    shape_path:
        Path to the STEP file on disk.
    """
    reader = STEPControl_Reader()
    status = reader.ReadFile(shape_path)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Не удалось прочитать STEP-файл: статус {status}")

    ok = reader.TransferRoots()
    if ok == 0:
        raise RuntimeError("Не удалось перенести корневые объекты из STEP")

    return reader.OneShape()
