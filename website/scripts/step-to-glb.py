"""Convert the hand STEP assembly into the website's rigged GLB model.

    uv run --with cadquery-ocp --with pygltflib python website/scripts/step-to-glb.py

(Run from the repo root, in the project env: it also needs `mujoco` and `numpy`.)

1. Reads hardware/cad/Hand assebly.step, meshes every part with OpenCascade
   and writes website/public/models/hand.glb (metres, glTF Y-up).
2. Rigs it for the site (see website/public/models/README.md): each STEP part
   is matched to a body of the MuJoCo model (sim/models/tendra_hand.xml) by its
   bounding box, and gets a parent node named after its joint, placed at the
   joint pivot with local X = the joint axis (positive = closing the hand, the
   same signs as the sim). `<finger>_tip` nodes mark the fingertips.

The STEP and the URDF export must come from the same Fusion design (same
frame). Re-run after exporting a new STEP; re-run sim/convert.py first if the
URDF changed.
"""

import struct
import sys
from pathlib import Path

import mujoco
import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.collections import IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString as FileInfo
from OCP.collections import Sequence_TDF_Label as LabelSequence
from OCP.IFSelect import IFSelect_RetDone
from OCP.Message import Message_ProgressRange
from OCP.RWGltf import RWGltf_CafWriter
from OCP.RWMesh import RWMesh_CoordinateSystem
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from pygltflib import GLTF2, Node

ROOT = Path(__file__).resolve().parents[2]
STEP = ROOT / "hardware/cad/Hand assebly.step"
MJCF = ROOT / "sim/models/tendra_hand.xml"
GLB = ROOT / "website/public/models/hand.glb"

# Mesh detail: chord deviation in model units (mm) and angle in radians.
# Coarser = smaller file. Keep the GLB under ~3 MB (it loads on the homepage).
LINEAR_DEFLECTION = 0.08
ANGULAR_DEFLECTION = 0.35

# A STEP part matches a sim body when their bounding boxes agree within this (metres).
MATCH_TOLERANCE = 0.001

# Sim/URDF frame (Z up, palm facing -Y) -> glTF (Y up, palm facing +Z). A proper rotation,
# so rotation axes and their signs carry over unchanged.
URDF_TO_GLTF = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)


def export_glb() -> None:
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(STEP)) != IFSelect_RetDone:
        sys.exit(f"Could not read {STEP}")
    reader.Transfer(doc)

    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free = LabelSequence()
    shapes.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        BRepMesh_IncrementalMesh(shapes.GetShape_s(free.Value(i)), LINEAR_DEFLECTION, False, ANGULAR_DEFLECTION, True)

    GLB.parent.mkdir(parents=True, exist_ok=True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(str(GLB)), True)
    writer.SetMergeFaces(True)  # one mesh per part instead of one per CAD face (~700 → 9 draw calls)
    # Fusion's STEP is Z-up in millimetres; glTF is Y-up in metres.
    converter = writer.ChangeCoordinateSystemConverter()
    converter.SetInputLengthUnit(0.001)
    converter.SetInputCoordinateSystem(RWMesh_CoordinateSystem.RWMesh_CoordinateSystem_Zup)
    converter.SetOutputCoordinateSystem(RWMesh_CoordinateSystem.RWMesh_CoordinateSystem_glTF)
    if not writer.Perform(doc, FileInfo(), Message_ProgressRange()):
        sys.exit("glTF export failed")


# ---------------------------------------------------------------- glTF helpers


def quat_from_matrix(r: np.ndarray) -> list[float]:
    """3x3 rotation matrix -> glTF quaternion [x, y, z, w]."""
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([x, y, z, w])
    return (q / np.linalg.norm(q)).tolist()


def matrix_from_quat(q) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def node_matrix(node: Node) -> np.ndarray:
    m = np.eye(4)
    if node.matrix:
        return np.array(node.matrix, dtype=float).reshape(4, 4).T  # glTF is column-major
    if node.rotation:
        m[:3, :3] = matrix_from_quat(node.rotation)
    if node.scale:
        m[:3, :3] = m[:3, :3] * np.array(node.scale)
    if node.translation:
        m[:3, 3] = node.translation
    return m


def set_node_matrix(node: Node, m: np.ndarray) -> None:
    """Writes a rigid transform as TRS (the site moves joint nodes through .position)."""
    node.matrix = None
    node.scale = None
    node.translation = m[:3, 3].tolist()
    node.rotation = quat_from_matrix(m[:3, :3])


def mesh_vertices(gltf: GLTF2, mesh_index: int) -> np.ndarray:
    """All POSITION vertices of a mesh (float32 VEC3), in the mesh's own frame."""
    blob = gltf.binary_blob()
    out = []
    for prim in gltf.meshes[mesh_index].primitives:
        acc = gltf.accessors[prim.attributes.POSITION]
        view = gltf.bufferViews[acc.bufferView]
        start = (view.byteOffset or 0) + (acc.byteOffset or 0)
        stride = view.byteStride or 12
        for i in range(acc.count):
            out.append(struct.unpack_from("<3f", blob, start + i * stride))
    return np.array(out)


def transform(m: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ m[:3, :3].T + m[:3, 3]


# ---------------------------------------------------------------- sim model


def sim_bodies():
    """Bodies of the sim model at q = 0: bounding box, parent, joint (in glTF coordinates)."""
    m = mujoco.MjModel.from_xml_path(str(MJCF))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    name = lambda kind, i: mujoco.mj_id2name(m, kind, i)  # noqa: E731
    bodies: dict[str, dict] = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        verts = m.mesh_vert[a : a + n] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        b = m.geom_bodyid[g]
        bodies[name(mujoco.mjtObj.mjOBJ_BODY, b)] = {
            "id": b,
            "box": np.array([verts.min(0), verts.max(0)]) @ URDF_TO_GLTF.T,
            "parent": name(mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[b]),
            "joint": None,
        }
    for j in range(m.njnt):
        body = name(mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])
        bodies[body]["joint"] = {
            "name": name(mujoco.mjtObj.mjOBJ_JOINT, j),
            "pivot": URDF_TO_GLTF @ d.xanchor[j],
            "axis": URDF_TO_GLTF @ d.xaxis[j],
        }
    for b in bodies.values():  # the box above was rotated corner by corner; re-sort it
        b["box"] = np.array([b["box"].min(0), b["box"].max(0)])
    return bodies


# ---------------------------------------------------------------- rigging


def joint_frame(pivot: np.ndarray, axis: np.ndarray, toward: np.ndarray) -> np.ndarray:
    """World matrix of a joint node: origin at the pivot, X = axis, Y toward the fingertip.

    Z = X × Y is then the way the finger moves when the joint closes, i.e. the
    palm side, which is where the site expects local +Z.
    """
    x = axis / np.linalg.norm(axis)
    y = toward - pivot
    y = y - x * (y @ x)
    if np.linalg.norm(y) < 1e-6:  # fingertip on the axis: any perpendicular will do
        y = np.cross(x, [0, 0, 1] if abs(x[2]) < 0.9 else [1, 0, 0])
    y /= np.linalg.norm(y)
    m = np.eye(4)
    m[:3, :3] = np.column_stack([x, y, np.cross(x, y)])
    m[:3, 3] = pivot
    return m


def rig() -> None:
    gltf = GLTF2().load(str(GLB))
    root_index = gltf.scenes[gltf.scene or 0].nodes[0]
    root = gltf.nodes[root_index]
    root_world = node_matrix(root)
    bodies = sim_bodies()

    # 1. Match every STEP part to a sim body by bounding box.
    parts: dict[str, dict] = {}  # body name -> {"node": index, "world": 4x4, "verts": world vertices}
    for idx in list(root.children):
        node = gltf.nodes[idx]
        if node.mesh is None:
            continue
        world = root_world @ node_matrix(node)
        verts = transform(world, mesh_vertices(gltf, node.mesh))
        box = np.array([verts.min(0), verts.max(0)])
        body, err = min(((b, np.abs(box - v["box"]).max()) for b, v in bodies.items()), key=lambda t: t[1])
        if err > MATCH_TOLERANCE or body in parts:
            print(f"  ! part {node.name!r} has no sim body (off by {err * 1000:.1f} mm); kept fixed on the palm")
            continue
        print(f"  {node.name:28s} -> {body}")
        node.extras = {"cad_name": node.name}
        node.name = body
        parts[body] = {"node": idx, "world": world, "verts": verts}
    missing = sorted(set(bodies) - set(parts))
    if missing:
        sys.exit(f"No STEP part found for sim bodies {missing}. Are the STEP and the URDF from the same design?")

    # 2. Fingertips: the far end of each chain's last segment.
    children = {b: [c for c, v in bodies.items() if v["parent"] == b] for b in bodies}
    tips: dict[str, np.ndarray] = {}  # finger -> fingertip point
    for body, info in bodies.items():
        if children[body] or not info["joint"]:
            continue
        finger = info["joint"]["name"].split("_")[0]
        chain = [body]
        while bodies[chain[-1]]["parent"] in bodies and bodies[bodies[chain[-1]]["parent"]]["joint"]:
            chain.append(bodies[chain[-1]]["parent"])
        base = bodies[chain[-1]]["joint"]["pivot"]
        verts = parts[body]["verts"]
        reach = np.linalg.norm(verts - base, axis=1)
        tips[finger] = verts[reach > reach.max() - 0.002].mean(0)  # centre of the last 2 mm

    # 3. Joint nodes, parents before children; each part hangs under its own joint.
    worlds: dict[str, np.ndarray] = {}  # body name -> world matrix of the node its part hangs under
    parent_index: dict[str, int] = {}  # body name -> glTF node index that its children attach to
    root.children = []
    order = [b for b in bodies if bodies[b]["parent"] not in bodies]
    for b in order:
        order.extend(c for c in children[b] if c not in order)
    for body in order:
        info = bodies[body]
        parent = info["parent"]
        part = gltf.nodes[parts[body]["node"]]
        if info["joint"] is None:  # the palm: fixed, straight under the root
            worlds[body], parent_index[body] = root_world, root_index
            root.children.append(parts[body]["node"])
            continue
        j = info["joint"]
        finger = j["name"].split("_")[0]
        world = joint_frame(j["pivot"], j["axis"], tips[finger])
        joint = Node(name=j["name"], children=[parts[body]["node"]])
        set_node_matrix(joint, np.linalg.inv(worlds[parent]) @ world)
        gltf.nodes.append(joint)
        idx = len(gltf.nodes) - 1
        gltf.nodes[parent_index[parent]].children.append(idx)
        set_node_matrix(part, np.linalg.inv(world) @ parts[body]["world"])
        worlds[body], parent_index[body] = world, idx
        if not children[body]:  # last segment: add the fingertip marker
            tip = Node(name=f"{finger}_tip", translation=(np.linalg.inv(world) @ np.append(tips[finger], 1))[:3].tolist())
            gltf.nodes.append(tip)
            joint.children.append(len(gltf.nodes) - 1)
        print(f"  joint {j['name']:15s} at {np.round(j['pivot'] * 1000, 1)} mm, axis {np.round(j['axis'], 3)}")

    gltf.save_binary(str(GLB))


def main() -> None:
    export_glb()
    print("Rigging:")
    rig()
    print(f"Wrote {GLB.relative_to(ROOT)} ({GLB.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
