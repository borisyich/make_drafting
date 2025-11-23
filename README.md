# afr3d – 3D-to-Drafting & Auto-Dimensioning

This project explores how to automatically generate technical drawings and dimensions from 3D CAD models using **OpenCascade (OCCT)** and **pythonOCC**.

The main ideas:

- Use OCCT as the geometric kernel (no dependence on GUI tools like NX/TechDraw during core processing).
- Find stable front/top/side views from the 3D model automatically.
- Extract geometric features (holes, cylinders, steps, etc.) as parameters.
- Build a *minimal but sufficient* set of dimensions using a constraint/DOF-based approach.
- Optionally, plug this into a reinforcement-learning (RL) environment where an agent decides which views and dimensions to use.

---

Basic part analysis: 
1 conda activate occ_env
2 C:\Users\dreck\Documents\make_drafting\src>python -m afr3d ..\data\example_complex.stp
'''
---

## 0. Technology Stack

- **OCCT + pythonOCC** – geometric kernel and 3D topology:
  - `BRepBndLib::AddOBB` – oriented bounding box (OBB).
  - `HLRBRep_Algo` / `HLRBRep_PolyAlgo` – hidden line removal (HLR), visible/hidden edges.
- **Constraint solving for auto-dimensions** – options:
  - External solvers (e.g. FreeCAD Sketcher, SolveSpace), or
  - Custom DOF counter on a parameter graph.
- **Rendering** – any 2D library is acceptable:
  - Simple: Cairo / Matplotlib.
  - Rich UI: FreeCAD/Qt, or another viewer.  
  Rendering is not a hard requirement; it is just a way to visualize the final result.

---

## 1. Import STEP and Compute OBB

1. Load the `.step` file into a `TopoDS_Shape` using pythonOCC.
2. Call `BRepBndLib::AddOBB(shape, obb)` to compute the oriented bounding box.

From the OBB we get:

- Center
- Three axes: `d1`, `d2`, `d3`
- Half-sizes: `sx`, `sy`, `sz`

These axes give us natural candidate directions for standard views.

---

## 2. Choosing Front/Top/Side Views

Candidate view directions are six vectors: `±d1`, `±d2`, `±d3`.

For each axis we compute a “projected area” based on OBB half-sizes:

- For directions along `d1`: `A_obb(±d1) = sy * sz`
- Along `d2`: `A_obb(±d2) = sx * sz`
- Along `d3`: `A_obb(±d3) = sx * sy`

We pick 2–3 directions with the largest `A_obb` as candidates for the **front view**.

For each candidate:

1. Run `HLRBRep_Algo` to generate visible/hidden segments.
2. Compute:
   - `L_vis` – total length of visible edges
   - `L_hid` – total length of hidden edges
   - `R_hidden = L_hid / (L_vis + eps)`

Then:

- **Front view** – direction with maximal `A_obb`, and if values are close, we choose the one with **minimal** `R_hidden`.
- **Top/Side views** – from remaining directions that are orthogonal to the chosen front:
  - Again use a combination of projected area and `R_hidden`.
  - The remaining orthogonal direction becomes the side view.

This way we do **not** depend on temporary drawing views in CAD. Everything is based purely on OBB + HLR.

---

## 3. Building 2D Views via HLR

For the selected front/top/side view directions we define view normals:

- `n_f`, `n_t`, `n_s` – normals of front, top, side views.

We build a local object coordinate system:

- `z_obj = -n_f`
- `y_obj = -n_t`
- `x_obj = normalize(y_obj × z_obj)`

For the **front view**:

- Horizontal axis: `u_f = x_obj`
- Vertical axis: `v_f = y_obj`

Top and side views are defined in a similar way, with consistent “up/right” orientation.

For each view we:

1. Define the viewing direction and projection plane.
2. Run HLR to get a set of 2D curves in `(ξ, η)` coordinates:
   - Lines, arcs, etc.
   - Visibility flags: *visible* / *hidden*.

These 2D curves are our raw linework for the drawing views.

---

## 4. Extracting 3D Parameters (Features)

From the 3D shape we extract:

- **Bounding box** in the local object frame `(x_obj, y_obj, z_obj)`:
  - Gives overall dimensions `L, W, H`.
- **Cylindrical faces** – holes and shafts:
  - Axis, radius, depth.
- **Planar faces** – steps, walls, shelves, etc.

For each feature type we define a standard parameter template. For example:

- Hole:
  - Diameter `Ø`
  - Coordinates `(x, y)` from base planes.
- Hole groups:
  - Diameter `Ø`
  - Pitch (step) in X/Y
  - Offsets from base references.
- Steps:
  - Diameter
  - Width
  - Axial position, etc.

As a result, we get a parameter list `p1…pn` and constraints between parameters.

---

## 5. Auto-Dimensioning: Minimal Parameter Set

We represent parameters and constraints as a **graph of DOF (degrees of freedom)**:

- Each dimension/constraint reduces DOF count.
- The goal is a **complete but not redundant** parameter set.

A simple greedy strategy:

1. Start with:
   - 3 overall dimensions: `L`, `W`, `H`.
   - Diameters of all unique holes.
   - Coordinates of hole centers (2 dimensions per hole or per hole group).
2. Compute remaining DOF (similar to work by Serrano and others on automatic dimension placement).
3. If DOF is not yet ~0, add:
   - Step dimensions,
   - Wall thicknesses,
   - Other feature parameters.

At this stage we already have a full description of the part. It may be slightly redundant, which is acceptable for an MVP.

For each parameter we also define:

- On which **view** it should appear (front/top/side).
- Which 2D points/lines are used for the dimension.
- The dimension type (linear / diameter / radius).

---

## 6. Dimension Placement and Rendering

For each 2D view we now have a structured description:

- 2D curves (lines, arcs, circles).
- Reference points (nodes, hole centers).
- Dimensions attached to these references.

We can:

- Use a custom layout algorithm with simple rules:
  - Overall dimensions outside the part.
  - Local dimensions closer to the related geometry.
- Or integrate with an existing drawing system (e.g. FreeCAD TechDraw), similar to “Auto Dimension” features in commercial CAD.

Finally, we render:

- Three separate PNGs (front/top/side), or
- A single drawing sheet with all views.

Line styles:

- Visible edges – solid.
- Hidden edges – dashed.
- Dimensions – arrows, extension lines, and text.

---

## 7. RL / Agent Environment (Optional)

With this architecture, the RL environment becomes simple and modular.

**Environment functions:**

- `get_candidate_views()` – returns OBB-based directions and rough metrics.
- `project(view_dir)` – runs HLR and returns 2D curves for that direction.
- `extract_features()` – returns detected features and their parameters.
- `simulate_dimensions(subset_of_params)` – checks if the subset of parameters fully defines the part (via solver) and returns:
  - complete / incomplete / redundant.
- `render_state()` – renders the current drawing state (for debugging or training).

**Agent actions:**

- Choose front/top/side views (currently hard-coded, but can be learned).
- Decide which parameters to include or exclude.
- Decide which dimensions belong on which views.

Heavy tasks remain in dedicated modules:

- Geometry – OCCT.
- Constraint solving – external solvers or a custom solver.

The agent focuses on **semantic choices**, not on low-level geometry math.

---

## Suggested Project Structure

Supposed project layout:

```text
afr3d/
├─ pyproject.toml            # or setup.cfg, if you prefer
├─ src/
│  └─ afr3d/
│     ├─ __init__.py
│     ├─ io/
│     │  ├─ __init__.py
│     │  └─ step_import.py        # load_step_shape(path) -> TopoDS_Shape
│     ├─ topo/
│     │  ├─ __init__.py
│     │  └─ topology.py           # face collection, adjacency graph, OBB, etc.
│     ├─ geom/
│     │  ├─ __init__.py
│     │  └─ primitives.py         # PlanarFaceInfo, CylindricalFaceInfo, ConicalFaceInfo, gp_* helpers
│     ├─ features/
│     │  ├─ __init__.py
│     │  ├─ cylinders.py          # extract_cylindrical_faces, CylindricalFeature, clustering
│     │  ├─ fillets.py            # edge graph, detect_fillet_candidates, basic fillet masks
│     │  ├─ holes.py              # HoleAFR, ChamferInfo, detect_holes_afr
│     │  ├─ pockets.py            # placeholder for pockets
│     │  ├─ slots.py              # placeholder for slots
│     │  └─ steps.py              # placeholder for simple steps
│     ├─ views/
│     │  ├─ __init__.py
│     │  ├─ orientation.py        # oriented bbox, base axes, main view selection
│     │  ├─ projections.py        # wrappers for OCCT 3D -> 2D HLR
│     │  └─ dimensioning.py       # auto-dimension logic
│     └─ utils/
│        ├─ __init__.py
│        ├─ debug.py              # debug_face_neighbors, text-based inspection
│        └─ logging.py            # lightweight logging helpers
└─ notebooks/
   ├─ 01_playground.ipynb         # general experiments
   ├─ 02_holes_debug.ipynb        # debugging hole detection
   └─ 03_dimensioning.ipynb       # auto-dimensioning prototype
```

This structure is only a suggestion. You can simplify or extend it depending on your needs.
