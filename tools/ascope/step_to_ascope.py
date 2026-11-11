"""Converts the robot's Onshape STEP export into an AdvantageScope robot model.

Writes ascope_assets/Robot_3256/: model.glb (everything that doesn't move), model_N.glb for each
articulated component, and config.json. Each mesh is written in the robot frame (meters, +X
forward, +Y left, +Z up, origin at robot center on the floor) with each component's origin at its
pivot, so the robot code just logs component poses (see frc.robot.RobotVisualizer) and the config
needs no correction rotations.

Setup (once):
    uv venv --python 3.12 ~/.venvs/cad
    VIRTUAL_ENV=~/.venvs/cad uv pip install "cadquery-ocp>=7.8,<7.9" trimesh numpy

Usage:
    ~/.venvs/cad/bin/python tools/ascope/step_to_ascope.py ~/Downloads/"Real Real Main.step"

The first run tessellates the STEP (about a minute) and caches the meshes next to it; later runs
reuse the cache, so regrouping parts is quick. Pass --retessellate after a new export.

Which parts move with which mechanism is set in COMPONENTS below. When a part ends up in the wrong
group, add its name (or a unique piece of it) to the right list.
"""

import argparse
import json
import pickle
import re
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "ascope_assets" / "Robot_3256"

# CAD frame -> robot frame. In the CAD the intake is on -Y and Z is up. The intake is on the back
# of the robot (the autos drive intake-first, backwards), and the turret and front-right camera
# land where TurretConstants and VisionConstants put them with robot +X (forward) = CAD +Y and
# robot +Y (left) = CAD -X.
CAD_TO_ROBOT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
MM_TO_M = 0.001

# Mesh resolution. Coarse enough to keep AdvantageScope smooth (holes become polygons).
LINEAR_DEFLECTION_MM = 1.5
ANGULAR_DEFLECTION_RAD = 0.6
# Parts bigger than MIN_TRIANGLES are decimated to KEEP_FRACTION of their triangles (but never
# below MIN_TRIANGLES), which mostly thins out the hole patterns in the plates and tubes.
MIN_TRIANGLES = 1500
KEEP_FRACTION = 0.15

# Parts dropped from every model: hardware nobody needs to see, and bodies whose geometry is not the
# part itself (the ThriftyCams carry their field-of-view cone as a solid).
EXCLUDE = [
    r"screw", r"\bnut\b", r"locknut", r"bolt", r"shcs", r"bhcs", r"heatset", r"washer", r"rivet", r"zip", r"snap ?ring",
    r"bushing", r"bearing", r"spacer", r"collar", r"origin cube", r"thriftycam \(",
]
# Anything reaching farther than this from the robot center (meters), or below the floor, is a stray
# body, not robot. The deployed intake reaches 0.65 m.
MAX_EXTENT = 0.70
MAX_HEIGHT = 0.80
MIN_HEIGHT = -0.005

# Articulated components, in the order the robot code logs their poses (see
# frc.robot.RobotVisualizer). A part belongs to a component when its path contains one of "within"
# and its leaf name contains one of "parts" (case-insensitive; None means any part), and, if given,
# "where" accepts its robot-frame bounding box center. Components are tried in MATCH_ORDER, so the
# hood claims its parts before the turret takes the rest of the shooter. "pivot" is the component
# origin in CAD millimeters.
COMPONENTS = [
    {
        "name": "Turret",
        "rules": [
            {"within": ["0900_V2_Shooter_Assembly"], "parts": None},
            {
                "within": ["0200_Turret_Assembly"],
                "parts": [
                    "0202_Turret_Ring", "0203_Outer_Clamping_Plate", "0204_Inner_Clamping_Plate",
                    "0205_Big_Gear", "0218_IGUS_Wrap_Tunnel",
                ],
            },
        ],
        # The turret gear's center.
        "pivot": [146.05, 146.05, 0.0],
    },
    {
        "name": "Hood",
        "rules": [
            {
                "within": ["0900_V2_Shooter_Assembly"],
                "parts": [
                    "0902_Left_Pivot_Plate", "0906_Right_Pivot_Plate", "0909_Hood_Gear_Plate",
                    "0910_Hood_Rack_Gear", "0925_Small_Pivot_Hardstop",
                ],
            },
        ],
        # The flywheel axle (center of the four flywheel disc stacks), which the hood swings about.
        "pivot": [168.2, 223.1, 595.3],
    },
    {
        "name": "Intake",
        "rules": [
            # Everything on the intake arm sits behind the frame (robot x < -0.343 m) with the
            # intake down, as in the CAD; the pivot motor, gearbox and jackshaft stay inside.
            {"within": ["0550_Ground_Intake_Assembly"], "parts": None, "where": lambda c: c[0] < -0.343},
        ],
        # The 0500_Pivot_Axle.
        "pivot": [0.0, -274.5, 216.0],
    },
]
MATCH_ORDER = [1, 0, 2]


def tessellate(step_path, cache_path):
    """Reads the STEP and returns a list of leaf parts with their world-space mesh and color."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import (
        XCAFDoc_ColorGen,
        XCAFDoc_ColorSurf,
        XCAFDoc_ColorTool,
        XCAFDoc_DocumentTool,
    )

    print(f"Reading {step_path} ...", flush=True)
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        sys.exit("Could not read the STEP file")
    reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(label):
        attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return attr.Get().ToExtString()
        return ""

    def color_of(labels, shape):
        c = Quantity_Color()
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if colors.GetColor(shape, kind, c):
                return (c.Red(), c.Green(), c.Blue())
            for label in labels:
                if XCAFDoc_ColorTool.GetColor_s(label, kind, c):
                    return (c.Red(), c.Green(), c.Blue())
        return None

    parts = []

    def walk(label, location, path, inherited_color):
        referred = label
        if shapes.IsReference_s(label):
            referred = TDF_Label()
            shapes.GetReferredShape_s(label, referred)
            location = location.Multiplied(shapes.GetLocation_s(label))
        color = color_of([label, referred], shapes.GetShape_s(referred)) or inherited_color
        path = path + [name_of(label) or name_of(referred)]
        kids = TDF_LabelSequence()
        shapes.GetComponents_s(referred, kids)
        if kids.Length() > 0:
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), location, path, color)
            return

        shape = shapes.GetShape_s(referred).Moved(location)
        BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION_MM, False, ANGULAR_DEFLECTION_RAD, True)
        vertices, faces = [], []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            loc = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, loc)
            if triangulation is not None:
                trsf = loc.Transformation()
                base = len(vertices)
                for i in range(1, triangulation.NbNodes() + 1):
                    p = triangulation.Node(i).Transformed(trsf)
                    vertices.append((p.X(), p.Y(), p.Z()))
                reversed_face = face.Orientation() == TopAbs_REVERSED
                for i in range(1, triangulation.NbTriangles() + 1):
                    a, b, c = triangulation.Triangle(i).Get()
                    if reversed_face:
                        b, c = c, b
                    faces.append((base + a - 1, base + b - 1, base + c - 1))
            explorer.Next()
        if faces:
            parts.append(
                {
                    "path": path,
                    "vertices": np.asarray(vertices, dtype=np.float32),
                    "faces": np.asarray(faces, dtype=np.int32),
                    "color": color or (0.6, 0.6, 0.6),
                }
            )

    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [], None)
    print(f"Tessellated {len(parts)} parts", flush=True)
    with open(cache_path, "wb") as f:
        pickle.dump(parts, f)
    return parts


def matches(text, patterns):
    text = text.lower()
    return any(p.lower() in text for p in patterns)


def group_of(part):
    leaf = part["path"][-1]
    vertices = to_robot_frame(part["vertices"], [0, 0, 0])
    center = (vertices.min(0) + vertices.max(0)) / 2
    for index in MATCH_ORDER:
        for rule in COMPONENTS[index]["rules"]:
            if not any(w in p for w in rule["within"] for p in part["path"]):
                continue
            if rule["parts"] is not None and not matches(leaf, rule["parts"]):
                continue
            if "where" in rule and not rule["where"](center):
                continue
            return index
    return None


def to_robot_frame(vertices_mm, pivot_mm):
    return ((vertices_mm - np.asarray(pivot_mm)) * MM_TO_M) @ CAD_TO_ROBOT.T


def simplify(vertices, faces):
    """Welds the part's vertices and decimates big parts down toward KEEP_FRACTION."""
    import fast_simplification

    vertices, inverse = np.unique(np.round(vertices, 4), axis=0, return_inverse=True)
    faces = inverse.reshape(-1)[faces]
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    if len(faces) <= MIN_TRIANGLES:
        return vertices, faces
    target = max(MIN_TRIANGLES, KEEP_FRACTION * len(faces))
    vertices, faces = fast_simplification.simplify(
        vertices.astype(np.float32), faces.astype(np.int32), target_reduction=1 - target / len(faces)
    )
    return vertices, faces


def build_mesh(parts, pivot_mm):
    """Merges the parts into one mesh per color, each with a plain PBR material."""
    by_color = {}
    for part in parts:
        key = tuple(round(c, 3) for c in part["color"])
        by_color.setdefault(key, []).append(part)
    scene = trimesh.Scene()
    for index, (color, same_color) in enumerate(by_color.items()):
        vertices, faces, offset = [], [], 0
        for part in same_color:
            v, f = simplify(to_robot_frame(part["vertices"], pivot_mm), part["faces"])
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
        mesh = trimesh.Trimesh(np.vstack(vertices), np.vstack(faces), process=False)
        # Without normals AdvantageScope lights the model as pure black. Degenerate triangles leave
        # some vertices with a zero normal, which glTF rejects, so point those up.
        normals = np.array(mesh.vertex_normals)
        zero = np.linalg.norm(normals, axis=1) < 1e-6
        normals[zero] = [0.0, 0.0, 1.0]
        mesh.vertex_normals = normals
        mesh.visual = trimesh.visual.TextureVisuals(
            material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[*color, 1.0], metallicFactor=0.2, roughnessFactor=0.6
            )
        )
        scene.add_geometry(mesh, geom_name=f"color_{index}")
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("step", type=Path)
    parser.add_argument("--retessellate", action="store_true")
    parser.add_argument("--list", action="store_true", help="print every part and its group")
    args = parser.parse_args()

    cache = args.step.with_suffix(".ascope-cache.pkl")
    if cache.exists() and not args.retessellate and cache.stat().st_mtime > args.step.stat().st_mtime:
        with open(cache, "rb") as f:
            parts = pickle.load(f)
    else:
        parts = tessellate(args.step, cache)

    kept = []
    seen = set()
    for part in parts:
        leaf = part["path"][-1]
        if any(re.search(p, leaf, re.IGNORECASE) for p in EXCLUDE):
            continue
        robot = to_robot_frame(part["vertices"], [0, 0, 0])
        if (
            np.abs(robot[:, :2]).max() > MAX_EXTENT
            or robot[:, 2].max() > MAX_HEIGHT
            or robot[:, 2].min() < MIN_HEIGHT
        ):
            print(f"  skipping stray body {' / '.join(part['path'])}")
            continue
        # Some subassemblies are included twice (e.g. the turret, under Drivebase Assembly too)
        key = (leaf, tuple(np.round(robot.min(0), 3)), tuple(np.round(robot.max(0), 3)))
        if key in seen:
            continue
        seen.add(key)
        kept.append(part)

    groups = {None: []}
    for index in range(len(COMPONENTS)):
        groups[index] = []
    for part in kept:
        groups[group_of(part)].append(part)

    if args.list:
        for part in kept:
            g = group_of(part)
            print(f"{'base' if g is None else COMPONENTS[g]['name']:12s} {' / '.join(part['path'][1:])}")

    OUT.mkdir(parents=True, exist_ok=True)
    build_mesh(groups[None], [0, 0, 0]).export(OUT / "model.glb", include_normals=True)
    print(f"model.glb: {len(groups[None])} parts")
    config_components = []
    for index, component in enumerate(COMPONENTS):
        if not groups[index]:
            sys.exit(f"No parts matched component {component['name']}")
        build_mesh(groups[index], component["pivot"]).export(
            OUT / f"model_{index}.glb", include_normals=True
        )
        print(f"model_{index}.glb ({component['name']}): {len(groups[index])} parts")
        # The mesh is already centered on the pivot, so the robot code logs the pivot's pose.
        pivot = to_robot_frame(np.array([component["pivot"]], dtype=float), [0, 0, 0])[0]
        print(f"  pivot in robot frame (m): {[round(float(v), 4) for v in pivot]}")
        config_components.append({"zeroedRotations": [], "zeroedPosition": [0.0, 0.0, 0.0]})

    config = {
        "name": "3256",
        "isFTC": False,
        "rotations": [],
        "position": [0.0, 0.0, 0.0],
        "cameras": [],
        "components": config_components,
    }
    with open(OUT / "config.json", "w") as f:
        json.dump(config, f, indent=2)
        f.write("\n")
    for path in sorted(OUT.glob("*.glb")):
        print(f"  {path.name}: {path.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
