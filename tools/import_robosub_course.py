#!/usr/bin/env python3
"""Import the Blender RoboSub 2026 course export into stonefish_sim.

Source: <export>/robosub_course_layout_COMBINED.obj — the whole course fused at
its final laid-out positions.  This file, not the placement table in
STONEFISH_SETUP.md, is the authority for where things go: earlier export
revisions disagreed with the table (most visibly on the torpedo board's pose),
and some elements are posed in ways a yaw-only table cannot express (the bins
are tilted, not flat).  Reading the layout directly makes any such
disagreement moot - whatever the modeller last placed is what gets imported.

The per-element OBJs are deliberately NOT used: their geometry has drifted from
the layout (the standalone `bins_pipeline.obj` bins are 0.34 x 0.34 x 0.279 and
axis-aligned, the laid-out ones are 0.471 x 0.502 x 0.352 and tilted).

Stonefish binds exactly one <look> per body, so every element is split into one
body per material; each chunk of an element shares the element's
`world_transform`, so the pieces stay glued together.

Frame conversion.  The export is Z-up with Z=0 on the pool floor; Stonefish here
is X-fwd / Y-right / Z-down with the water surface at Z=0 and the floor at
POOL_FLOOR_Z.  Because the layout already carries every element's pose in its
vertices, one transform `Rz(90 deg)·Rx(180 deg)` + offset applies to everything:

    X_w = Y_b + COURSE_ORIGIN[0]
    Y_w = X_b + COURSE_ORIGIN[1]
    Z_w = POOL_FLOOR_Z - Z_b

which is `world_transform xyz="OX OY FLOOR" rpy="pi 0 pi/2"` on every body.

Usage:  tools/import_robosub_course.py <path-to-Stonefish_Export>
            [--extra-textures <dir>]... [--element-obj <element>=<dir>]...
"""

from __future__ import annotations

import collections
import math
import re
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PKG = REPO / "stonefish_sim"
DATA = PKG / "data" / "object_files" / "robosub_course"

LAYOUT = "robosub_course_layout_COMBINED.obj"

# Pool basin floor depth (object_files/robosub/Pool_PoolPlane.obj), surface Z=0.
POOL_FLOOR_Z = 3.432
# Where the layout's origin lands in the world, chosen so the gate opening is
# centred on Y=0 and sits 4 m in front of the default drone spawn (0, 0, 2.6).
COURSE_ORIGIN = (4.0, -1.57)

# The octagon is modelled 1.8 m up ("floats near the surface" in the guide).
# Lift it to the actual surface for this pool depth.
OCTAGON_SURFACE_LIFT = POOL_FLOOR_Z - 1.8

# Per-element pose overrides: (Blender-frame target X, Y, extra yaw in degrees),
# applied as a rigid rotate-about-own-centroid-then-translate on top of whatever
# pose the layout OBJ already has.  Kept separate from PLACEMENT/ELEMENT_ORDER so
# it is obvious any entry here is a deliberate, tracked departure from the
# source file.  If a future export regresses an element's pose, add an entry
# here rather than editing vertices by hand, and derive the fix from the obj
# file's own geometry - never from a render: Stonefish's free-look camera does
# not point along drone_orientation, so a screenshot is not reliable evidence
# for which way an element is actually facing.
#
# slalom: the export lays each WHITE-RED-WHITE set along the course (world X),
# so the vehicle would drive along the pipes instead of through the sets.
# Turned 90 deg to the left (world +X -> -Y; +90 deg about the Blender Z-up
# axis) about its own vertex centroid, which stays where it was: each set now
# lies across the course and the three sets follow each other along X.
POSE_OVERRIDE = {
    "slalom": (1.997449, 6.141378, 90.0),
}

# Elements whose icon-slot decal needs its face winding reversed, because the
# export authored it facing into the element instead of out of it.  Empty
# unless a specific element is confirmed backwards by the same kind of
# obj-geometry derivation described above.
FLIP_ICON_NORMAL = set()

DYNAMIC = {
    "restore_jar_1",
    "restore_jar_2",
    "restore_container_1",
    "restore_container_2",
}

STATIC_MATERIAL = "Neutral"
DYNAMIC_MATERIAL = "RoboPickup"

# Thickness given to flat icon decals that end up inside a dynamic body.  A
# zero-volume mesh part yields zero mass / NaN inertia in Stonefish's compound
# solid, which Bullet then rejects with "Overflow in AABB, object removed".
ICON_THICKNESS = 0.002

# How far a static icon decal gets nudged off the surface behind it, to avoid
# the z-fighting described in Chunk.nudge_along_normal.  torpedo_board's decal
# is exactly coplanar with its backing along its own (correct, unflipped) face
# normal - nothing protrudes further in that direction, so a few mm clears it.
ICON_NUDGE = 0.003

# An element whose boundary edges exceed this fraction of its edges counts as an
# open shell and gets double-sided visuals.  See the comment where it is used.
OPEN_SHELL_FRACTION = 0.05


def element_of(obj: str) -> str:
    """Map a layout object name onto the course element it belongs to."""
    if obj.startswith("bin_"):
        return obj                      # bin_1..bin_4: one element each, so that
                                        # each carries an independent icon slot
    if obj.startswith("mesh"):
        return "bins_pipeline"          # the 535 PVC pipe/fitting parts
    if obj.startswith("octagon"):
        return "octagon"                # ring + 4 hanging plates
    if obj.startswith("restore_table") or obj.startswith("basket"):
        return "restore_table"          # table + 2 baskets
    return {
        "pathmarker": "pathmarker_1",
        "pathmarker.001": "pathmarker_2",
        "jar": "restore_jar_1",
        "jar.001": "restore_jar_2",
        "container": "restore_container_1",
        "container.001": "restore_container_2",
    }.get(obj, obj)


# Emission order; also the order bodies appear in the scenario.
ELEMENT_ORDER = [
    "gate", "pathmarker_1", "slalom", "pathmarker_2",
    "bins_pipeline", "bin_1", "bin_2", "bin_3", "bin_4",
    "torpedo_board", "octagon", "restore_table",
    "restore_jar_1", "restore_jar_2", "restore_container_1", "restore_container_2",
]

# --------------------------------------------------------------------------- #
# Role images (RoboSub 2026 Team Handbook section 3.2)
#
# Every role image becomes its own "slot": a uniquely named look, so which image
# a slot shows can be changed in metadata/robosub_looks.scn without touching the
# meshes.  The course is fixed (no randomization); the handbook fixes WHICH
# images exist and how many of each, the groups below say where they can go.
#
#   Task 1 gate     3.2.2  "an image of Survey & Repair on one side (could be
#                           either side), Search & Rescue on the other"
#   Task 3 bins     3.2.4  "four bins.  Two for the Survey & Repair role and two
#                           for the Search & Rescue role"
#   Task 4 board    3.2.5  "There are two different versions of the board"
#   Task 5 octagon  3.2.6  "Hanging from the octagon, facing inward, are images
#                           of Survey, Repair, Search and Rescue"
#   Task 5 baskets  3.2.6  "At the bottom of the Survey & Repair basket is
#                           Warning and at the bottom of Search & Rescue is
#                           RedCross"
#   Task 5 items    3.2.6  "two different items for each role: Survey & Repair
#                           has NutBolt and Electric; Search & Rescue has Pill
#                           and BandAid"
#
# NOT randomized, because the handbook pins them down: the gate's Red/Black
# panel pattern ("Red, Right, Above" - a heading cue), the slalom's White/Red/
# White pipes, and the fact that both roles always appear exactly once on the
# gate / twice among the bins / once each on the octagon.
# --------------------------------------------------------------------------- #

ICON_GROUPS = [
    # (group name, elements contributing slots, handbook reference)
    ("gate_roles",    ["gate"],                                   "3.2.2"),
    ("bin_roles",     ["bin_1", "bin_2", "bin_3", "bin_4"],       "3.2.4"),
    ("torpedo_board", ["torpedo_board"],                          "3.2.5"),
    ("octagon_plates", ["octagon"],                               "3.2.6"),
    ("basket_floors", ["restore_table"],                          "3.2.6"),
    ("table_items",   ["restore_jar_1", "restore_jar_2",
                       "restore_container_1", "restore_container_2"], "3.2.6"),
]

# The torpedo board is the one group whose pool is not simply "the textures this
# element already uses": the handbook says two versions of the board exist, and
# the export ships only one.  Task4_ver1.png lives in the RoboNation task-design
# folder - pass it with --extra-textures to enable the swap.  Safe to randomize
# now that the board's decal is modelled on both faces (see --element-obj), so
# there is no orientation risk from swapping which texture shows.
TORPEDO_VERSIONS = ["Task4_ver1.png", "Task4_ver2.png"]


# --------------------------------------------------------------------------- #
# OBJ / MTL parsing
# --------------------------------------------------------------------------- #

class Obj:
    """A parsed OBJ: shared vertex pools + faces bucketed by (object, material)."""

    def __init__(self, path: Path):
        self.v: list[str] = []
        self.vt: list[str] = []
        self.vn: list[str] = []
        self.faces: dict[tuple[str, str], list[list[tuple]]] = collections.defaultdict(list)
        obj = mat = ""
        for line in path.read_text().splitlines():
            if line.startswith("v "):
                self.v.append(line)
            elif line.startswith("vt "):
                self.vt.append(line)
            elif line.startswith("vn "):
                self.vn.append(line)
            elif line.startswith("o "):
                obj = line[2:].strip()
            elif line.startswith("usemtl"):
                bits = line.split(None, 1)
                mat = bits[1].strip() if len(bits) > 1 else ""
            elif line.startswith("f "):
                face = []
                for tok in line.split()[1:]:
                    bits = (tok.split("/") + ["", ""])[:3]
                    face.append((
                        self._idx(bits[0], len(self.v)),
                        self._idx(bits[1], len(self.vt)) if bits[1] else None,
                        self._idx(bits[2], len(self.vn)) if bits[2] else None,
                    ))
                self.faces[(obj, mat)].append(face)

    @staticmethod
    def _idx(tok: str, n: int) -> int:
        i = int(tok)
        return i - 1 if i > 0 else n + i

    def xyz(self, i: int) -> tuple[float, float, float]:
        return tuple(float(c) for c in self.v[i].split()[1:4])


def parse_mtl(path: Path) -> dict[str, dict]:
    """newmtl name -> {'kd': (r,g,b) | None, 'map_kd': 'file.png' | None}."""
    mats: dict[str, dict] = {}
    cur = None
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "newmtl":
            cur = line.split(None, 1)[1].strip()
            mats[cur] = {"kd": None, "map_kd": None}
        elif cur is None:
            continue
        elif parts[0] == "Kd":
            mats[cur]["kd"] = tuple(float(v) for v in parts[1:4])
        elif parts[0] == "map_Kd":
            mats[cur]["map_kd"] = Path(line.split(None, 1)[1].strip()).name
    return mats


def slug(name: str) -> str:
    """A filesystem/XML-safe short name for an MTL material."""
    name = re.sub(r"\.\d+$", "", name.strip())      # Blender's ".001" suffixes
    name = re.sub(r"_icon$", "", name)
    if not name or re.fullmatch(r"[\d._]+", name):  # "0.752941_0.752941_..."
        return "plain"
    return re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_").lower()


# --------------------------------------------------------------------------- #
# Looks
# --------------------------------------------------------------------------- #

class LookTable:
    """Deduplicates looks across elements; keyed by texture or by colour."""

    def __init__(self) -> None:
        self.by_key: dict[tuple, str] = {}
        self.defs: list[str] = []

    def icon_slot(self, slot: str, tex: str) -> str:
        """The look of one role-image slot, with the image the export put there.

        One per slot and never deduplicated, so a slot's image can be changed by
        hand in metadata/robosub_looks.scn (keep objects/robosub_landmarks.scn
        in sync: its subtypes say which role each slot shows).
        """
        name = f"robo_icon_{slot}"
        self.defs.append(
            f'            <look name="{name}" gray="1.0" roughness="0.6" '
            f'texture="object_files/robosub_course/textures/{tex}"/>'
        )
        return name

    def get(self, spec: dict) -> str:
        tex = spec.get("map_kd")
        if tex:
            key = ("tex", tex)
            name = "robo_" + slug(Path(tex).stem)
            attrs = ('gray="1.0" roughness="0.6" '
                     f'texture="object_files/robosub_course/textures/{tex}"')
        else:
            # Unassigned faces (a bare "usemtl" line) fall back to PVC white.
            kd = spec.get("kd") or (0.9, 0.9, 0.9)
            key = ("rgb", tuple(round(c, 3) for c in kd))
            name = "robo_" + colour_name(kd)
            attrs = f'rgb="{kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}" roughness="0.55"'
        if key in self.by_key:
            return self.by_key[key]
        while name in self.by_key.values():
            name += "_x"
        self.by_key[key] = name
        self.defs.append(f'            <look name="{name}" {attrs}/>')
        return name


def colour_name(kd) -> str:
    named = {
        (1.0, 1.0, 1.0): "white",
        (0.0, 0.0, 0.0): "black",
        (1.0, 0.0, 0.0): "red",
        (0.9, 0.9, 0.9): "pvc_white",
        (1.0, 0.35, 0.0): "orange",
        (0.8, 0.8, 0.8): "grey",
    }
    key = tuple(round(c, 3) for c in kd)
    if key in named:
        return named[key]
    return "rgb_%02x%02x%02x" % tuple(int(round(c * 255)) for c in kd)


# --------------------------------------------------------------------------- #
# Mesh chunk construction
# --------------------------------------------------------------------------- #

class Chunk:
    """One element's faces for one look, as a standalone mesh."""

    def __init__(self, src: Obj, faces, shift=(0.0, 0.0, 0.0)):
        vmap: dict[int, int] = {}
        tmap: dict[int, int] = {}
        nmap: dict[int, int] = {}
        self.faces: list[list[tuple[int, int | None, int | None]]] = []
        for face in faces:
            out = []
            for vi, ti, ni in face:
                if vi not in vmap:
                    vmap[vi] = len(vmap)
                t = None
                if ti is not None:
                    if ti not in tmap:
                        tmap[ti] = len(tmap)
                    t = tmap[ti]
                n = None
                if ni is not None:
                    if ni not in nmap:
                        nmap[ni] = len(nmap)
                    n = nmap[ni]
                out.append((vmap[vi], t, n))
            self.faces.append(out)

        if any(shift):
            self.v = []
            for i in sorted(vmap, key=vmap.get):
                x, y, z = src.xyz(i)
                self.v.append(f"v {x + shift[0]:.6f} {y + shift[1]:.6f} {z + shift[2]:.6f}")
        else:
            self.v = [src.v[i] for i in sorted(vmap, key=vmap.get)]
        self.vt = [src.vt[i] for i in sorted(tmap, key=tmap.get)]
        self.vn = [src.vn[i] for i in sorted(nmap, key=nmap.get)]
        self.note = ""

    # -- geometry queries --------------------------------------------------- #

    def is_closed(self) -> bool:
        """True if this chunk is a closed manifold on its own (a solid)."""
        use: collections.Counter = collections.Counter()
        for f in self.faces:
            pts = [tuple(round(c, 5) for c in self.pt(v)) for v, _, _ in f]
            for a, b in zip(pts, pts[1:] + pts[:1]):
                use[(min(a, b), max(a, b))] += 1
        return all(c == 2 for c in use.values())

    def pt(self, i: int) -> tuple[float, float, float]:
        return tuple(float(c) for c in self.v[i].split()[1:4])

    def is_flat(self) -> bool:
        """True if every vertex shares a value on some axis - not necessarily Z.

        A decal glued to the side of an element (the torpedo board's vinyl
        print, constant in Y) is exactly as flat as one glued to a top face
        (constant in Z); nudge_along_normal doesn't care which axis, so this
        must not either, or the axis-specific ones silently never fire.
        """
        pts = self.pts()
        return any(len({round(p[axis], 6) for p in pts}) == 1 for axis in range(3))

    def pts(self) -> list[tuple[float, float, float]]:
        return [tuple(float(c) for c in v.split()[1:4]) for v in self.v]

    # -- geometry edits ----------------------------------------------------- #

    def double_sided(self) -> "Chunk":
        """A copy with every face mirrored, so it renders from both sides.

        Stonefish backface-culls, so an open shell modelled with outward normals
        (the bin walls, the basket walls, the icon quads on the bin floors)
        vanishes when the camera is inside it — which is exactly where the AUV
        looks from.  Used for <visual> only; <physical> keeps the original.
        """
        out = Chunk.__new__(Chunk)
        out.v, out.vt = list(self.v), list(self.vt)
        n = len(self.vn)
        out.vn = list(self.vn)
        for line in self.vn:
            x, y, z = (float(c) for c in line.split()[1:4])
            out.vn.append(f"vn {-x:.6f} {-y:.6f} {-z:.6f}")
        out.faces = list(self.faces) + [
            [(v, t, None if nn is None else nn + n) for v, t, nn in reversed(f)]
            for f in self.faces
        ]
        out.note = " (faces mirrored: double-sided visual)"
        return out

    def nudge_along_normal(self, distance: float) -> None:
        """Push a flat chunk `distance` along its face normal, in place.

        Some role-image decals in the export sit exactly coplanar with the
        backing panel behind them (verified: torpedo_board's vinyl face and its
        backing share a vertex Y of -0.00344 to 5 decimal places) instead of
        being offset slightly in front.  Two coplanar triangles are a coin flip
        for the depth buffer, so the icon disappears behind the (usually grey or
        white) backing at render time - "a white plane where the icon should
        be".  Nudging the icon a couple of millimetres along its own normal,
        which by this point in the pipeline points the correct way (see the
        Blender fix noted in the README), resolves the race outright.
        """
        p0, p1, p2 = (self.pts()[i] for i in (self.faces[0][0][0],
                                               self.faces[0][1][0],
                                               self.faces[0][2][0]))
        u = tuple(p1[k] - p0[k] for k in range(3))
        w = tuple(p2[k] - p0[k] for k in range(3))
        nx, ny, nz = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                      u[0] * w[1] - u[1] * w[0])
        ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / ln * distance, ny / ln * distance, nz / ln * distance
        self.v = [
            f"v {x + nx:.6f} {y + ny:.6f} {z + nz:.6f}" for x, y, z in self.pts()
        ]

    def extrude_flat(self, thickness: float) -> None:
        """Turn a flat triangle set into a thin closed solid, in place.

        The cap keeps its position and UVs; a copy is pushed `thickness` down in
        -Z and the boundary edges are stitched with side quads.  A flat normal is
        emitted per face: Stonefish's OBJ reader indexes `normals[nID-1]`
        unconditionally, so a mesh without `vn` lines segfaults it.
        """
        pts = self.pts()
        n = len(pts)
        self.v += [f"v {x:.6f} {y:.6f} {z - thickness:.6f}" for x, y, z in pts]
        all_pts = pts + [(x, y, z - thickness) for x, y, z in pts]

        use: collections.Counter = collections.Counter()
        for f in self.faces:
            idx = [v for v, _, _ in f]
            for a, b in zip(idx, idx[1:] + idx[:1]):
                use[(min(a, b), max(a, b))] += 1

        def dup(c):
            v, t, _ = c
            return (v + n, t, None)

        tris = [[(v, t, None) for v, t, _ in f] for f in self.faces]           # cap
        tris += [[dup(c) for c in reversed(f)] for f in self.faces]            # copy
        for f in self.faces:                                                   # walls
            for a, b in zip(f, f[1:] + f[:1]):
                if use[(min(a[0], b[0]), max(a[0], b[0]))] != 1:
                    continue
                tris.append([(a[0], a[1], None), dup(a), dup(b)])
                tris.append([(a[0], a[1], None), dup(b), (b[0], b[1], None)])

        vol = 0.0
        for tri in tris:
            p0, p1, p2 = (all_pts[c[0]] for c in tri)
            vol += (p0[0] * (p1[1] * p2[2] - p2[1] * p1[2])
                    - p1[0] * (p0[1] * p2[2] - p2[1] * p0[2])
                    + p2[0] * (p0[1] * p1[2] - p1[1] * p0[2])) / 6.0
        if vol < 0.0:  # cap was wound the other way round - flip everything
            tris = [list(reversed(tri)) for tri in tris]

        self.vn = []
        self.faces = []
        for tri in tris:
            p0, p1, p2 = (all_pts[c[0]] for c in tri)
            u = tuple(p1[k] - p0[k] for k in range(3))
            w = tuple(p2[k] - p0[k] for k in range(3))
            nx = u[1] * w[2] - u[2] * w[1]
            ny = u[2] * w[0] - u[0] * w[2]
            nz = u[0] * w[1] - u[1] * w[0]
            ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            self.vn.append(f"vn {nx / ln:.6f} {ny / ln:.6f} {nz / ln:.6f}")
            ni = len(self.vn) - 1
            self.faces.append([(c[0], c[1], ni) for c in tri])
        self.note = f" (flat decal extruded to {thickness * 1000:.0f} mm)"

    # -- output ------------------------------------------------------------- #

    def write(self, path: Path, header: str) -> None:
        lines = [f"# {header}{self.note}"]
        lines += self.v + self.vt + self.vn
        for f in self.faces:
            toks = []
            for v, t, n in f:
                ts = "" if t is None else str(t + 1)
                ns = "" if n is None else str(n + 1)
                toks.append(f"{v + 1}/{ts}/{ns}" if (ts or ns) else str(v + 1))
            lines.append("f " + " ".join(toks))
        path.write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------- #
# Scenario generation
# --------------------------------------------------------------------------- #

def replace_element_geometry(layout: Obj, elem: str, obj_path: Path) -> dict:
    """Swap `elem`'s faces in `layout` for a dedicated single-element re-export.

    The re-export must use the same axis convention as the main combined
    layout (checked by the caller, not assumed here) - this only handles
    position: the replacement is translated so its own XY centroid lands where
    `elem` was already positioned in the layout, and its own bottom (min Z)
    lands where `elem`'s was.  No rotation is applied, so the re-export's own
    orientation is trusted as-is.

    Returns the replacement file's own parse_mtl() dict, since its materials
    are not necessarily in the layout's .mtl.
    """
    old_idx = {v for (obj, _), faces in layout.faces.items() if obj == elem
               for f in faces for v, _, _ in f}
    if not old_idx:
        print(f"error: '{elem}' not found in the layout - nothing to replace")
        return {}
    old_pts = [layout.xyz(i) for i in old_idx]
    target_x = sum(p[0] for p in old_pts) / len(old_pts)
    target_y = sum(p[1] for p in old_pts) / len(old_pts)
    target_z = min(p[2] for p in old_pts)

    ov = Obj(obj_path)
    new_pts = [ov.xyz(i) for i in range(len(ov.v))]
    cx = sum(p[0] for p in new_pts) / len(new_pts)
    cy = sum(p[1] for p in new_pts) / len(new_pts)
    cz = min(p[2] for p in new_pts)
    dx, dy, dz = target_x - cx, target_y - cy, target_z - cz

    base_v = len(layout.v)
    layout.v += [f"v {x + dx:.6f} {y + dy:.6f} {z + dz:.6f}" for x, y, z in new_pts]
    base_t, base_n = len(layout.vt), len(layout.vn)
    layout.vt += ov.vt
    layout.vn += ov.vn

    for key in [k for k in layout.faces if k[0] == elem]:
        del layout.faces[key]
    for (obj, mat), faces in ov.faces.items():
        layout.faces[(elem, mat)] = [
            [(v + base_v, None if t is None else t + base_t,
              None if n is None else n + base_n) for v, t, n in f]
            for f in faces
        ]
    print(f"  override: {elem} geometry replaced from {obj_path}, "
          f"positioned at blender ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    return parse_mtl(obj_path.with_suffix(".mtl"))


def apply_pose_override(layout: Obj, elem: str, target_x: float, target_y: float,
                        dyaw_deg: float) -> None:
    """Rotate `elem`'s vertices by dyaw_deg about their own XY centroid, then
    translate that centroid to (target_x, target_y).  Mutates layout.v in place.
    Only meant for the single-object elements in POSE_OVERRIDE (elem must equal
    a layout `o` name directly, not a grouped element like "bins_pipeline")."""
    idx = sorted({v for (obj, _), faces in layout.faces.items() if obj == elem
                  for f in faces for v, _, _ in f})
    if not idx:
        print(f"error: pose override for '{elem}' matches no layout object")
        return
    pts = [layout.xyz(i) for i in idx]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    c, s = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))
    dx, dy = target_x - cx, target_y - cy
    for i, (x, y, z) in zip(idx, pts):
        rx = c * (x - cx) - s * (y - cy) + cx + dx
        ry = s * (x - cx) + c * (y - cy) + cy + dy
        layout.v[i] = f"v {rx:.6f} {ry:.6f} {z:.6f}"
    # Turn the normals with the geometry. They are appended as new entries
    # (the layout may share normals between objects) and the faces repointed.
    new_n: dict[int, int] = {}
    for (obj, _), faces in layout.faces.items():
        if obj != elem:
            continue
        for f in faces:
            for k, (v, t, n) in enumerate(f):
                if n is None:
                    continue
                if n not in new_n:
                    nx, ny, nz = (float(c) for c in layout.vn[n].split()[1:4])
                    layout.vn.append(
                        f"vn {c * nx - s * ny:.6f} {s * nx + c * ny:.6f} {nz:.6f}")
                    new_n[n] = len(layout.vn) - 1
                f[k] = (v, t, new_n[n])
    print(f"  override: {elem:20s} rotated {dyaw_deg:.1f} deg about own centroid, "
          f"moved to blender ({target_x:.3f}, {target_y:.3f})")


def element_boundary(src: Obj, faces_by_look: dict) -> float:
    """Fraction of an element's edges that are used by exactly one face.

    Position-keyed, because Blender duplicates vertices at material seams, so
    indices do not match across chunks.
    """
    use: collections.Counter = collections.Counter()
    for faces in faces_by_look.values():
        for f in faces:
            pts = [tuple(round(c, 5) for c in src.xyz(v)) for v, _, _ in f]
            for a, b in zip(pts, pts[1:] + pts[:1]):
                use[(min(a, b), max(a, b))] += 1
    if not use:
        return 0.0
    return sum(1 for c in use.values() if c == 1) / len(use)


def world_pivot(x_b: float, y_b: float, z_b: float) -> tuple[float, float, float]:
    return (y_b + COURSE_ORIGIN[0], x_b + COURSE_ORIGIN[1], POOL_FLOOR_Z - z_b)


RPY = f"{math.pi:.5f} 0.00000 {math.pi / 2:.5f}"


def mesh_xml(indent: str, phys: str, vis: str) -> str:
    out = (f'{indent}<physical>\n'
           f'{indent}  <mesh filename="object_files/robosub_course/{phys}" scale="1.0"/>\n'
           f'{indent}  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
           f'{indent}</physical>\n')
    out += (f'{indent}<visual>\n'
            f'{indent}  <mesh filename="object_files/robosub_course/{vis}" scale="1.0"/>\n'
            f'{indent}  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
            f'{indent}</visual>\n')
    return out


def body_xml(name: str, chunks, pivot, dynamic: bool) -> str:
    xyz = f'{pivot[0]:.4f} {pivot[1]:.4f} {pivot[2]:.4f}'
    mat = DYNAMIC_MATERIAL if dynamic else STATIC_MATERIAL

    if not dynamic:
        out = []
        for phys, vis, look in chunks:
            out.append(f'  <static name="{Path(phys).stem}" type="model">\n'
                       + mesh_xml("    ", phys, vis)
                       + f'    <material name="{mat}"/>\n'
                       f'    <look name="{look}"/>\n'
                       f'    <world_transform xyz="{xyz}" rpy="{RPY}"/>\n'
                       '  </static>\n')
        return "\n".join(out)

    # Dynamic pickup: compound so each material chunk keeps its own look.
    parts = []
    for phys, vis, look in chunks:
        part = Path(phys).stem.split("__", 1)[1]
        parts.append(f'    <external_part name="{part}" type="model" physics="submerged">\n'
                     + mesh_xml("      ", phys, vis)
                     + f'      <material name="{mat}"/>\n'
                     f'      <look name="{look}"/>\n'
                     '      <compound_transform xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
                     '    </external_part>\n')
    return (f'  <dynamic name="{name}" type="compound" physics="submerged">\n'
            + "".join(parts)
            + f'    <world_transform xyz="{xyz}" rpy="{RPY}"/>\n'
            '  </dynamic>\n')


def main() -> int:
    args = sys.argv[1:]
    extra_tex: list[Path] = []
    while "--extra-textures" in args:
        i = args.index("--extra-textures")
        if i + 1 >= len(args):
            print("error: --extra-textures needs a directory")
            return 2
        extra_tex.append(Path(args[i + 1]).expanduser())
        del args[i:i + 2]
    element_overrides: list[tuple[str, Path]] = []
    while "--element-obj" in args:
        i = args.index("--element-obj")
        if i + 1 >= len(args) or "=" not in args[i + 1]:
            print("error: --element-obj needs <element>=<dir containing "
                  "<element>.obj/.mtl>")
            return 2
        elem, _, folder = args[i + 1].partition("=")
        element_overrides.append((elem, Path(folder).expanduser()))
        del args[i:i + 2]
    if len(args) != 1:
        print(__doc__)
        return 2
    src = Path(args[0]).expanduser().resolve()
    if not (src / LAYOUT).is_file():
        print(f"error: {src / LAYOUT} not found")
        return 1

    layout = Obj(src / LAYOUT)
    mats = parse_mtl(src / (Path(LAYOUT).stem + ".mtl"))

    for elem, (tx, ty, dyaw) in POSE_OVERRIDE.items():
        apply_pose_override(layout, elem, tx, ty, dyaw)

    for elem, folder in element_overrides:
        mats.update(replace_element_geometry(layout, elem, folder / f"{elem}.obj"))

    # Wipe generated content but keep README.md - it documents this directory
    # and is checked in, not generated.
    if DATA.exists():
        for item in DATA.iterdir():
            if item.name == "README.md":
                continue
            shutil.rmtree(item) if item.is_dir() else item.unlink()
    (DATA / "textures").mkdir(parents=True, exist_ok=True)
    for png in sorted((src / "textures").glob("*.png")):
        shutil.copy2(png, DATA / "textures" / png.name)
    for extra in extra_tex:
        for png in sorted(extra.glob("*.png")):
            shutil.copy2(png, DATA / "textures" / png.name)
    have_tex = {p.name for p in (DATA / "textures").glob("*.png")}

    # Bucket faces by element and material first, so each element's textured
    # materials can be turned into individually addressable icon slots.
    by_elem_mat: dict[str, dict[str, list]] = collections.defaultdict(
        lambda: collections.defaultdict(list))
    for (obj, mat), faces in layout.faces.items():
        by_elem_mat[element_of(obj)][mat] += faces

    missing = set(by_elem_mat) - set(ELEMENT_ORDER)
    if missing:
        print(f"error: unmapped layout objects -> {sorted(missing)}")
        return 1

    icon_elements = {e for _, elems, _ in ICON_GROUPS for e in elems}
    looks = LookTable()
    grouped: dict[str, dict[str, list]] = collections.defaultdict(
        lambda: collections.defaultdict(list))
    slots: dict[str, list[tuple[str, str]]] = {}   # element -> [(slot, texture)]

    for elem, by_mat in by_elem_mat.items():
        textured = sorted(
            ((m, mats[m]["map_kd"]) for m in by_mat if mats.get(m, {}).get("map_kd")),
            key=lambda mt: mt[1],
        )
        elem_slots: list[tuple[str, str]] = []
        for i, (mat, tex) in enumerate(textured):
            if elem not in icon_elements:
                continue
            slot = elem if len(textured) == 1 else f"{elem}_{i + 1}"
            elem_slots.append((slot, tex))
            grouped[elem][looks.icon_slot(slot, tex)] += by_mat[mat]
        if elem_slots:
            slots[elem] = elem_slots
        swapped = {m for m, _ in textured} if elem in icon_elements else set()
        for mat, faces in by_mat.items():
            if mat not in swapped:
                grouped[elem][looks.get(mats.get(mat, {}))] += faces

    for elem in FLIP_ICON_NORMAL:
        for look in list(grouped[elem]):
            if look.startswith("robo_icon_"):
                grouped[elem][look] = [list(reversed(f)) for f in grouped[elem][look]]

    bodies = []
    for name in ELEMENT_ORDER:
        by_look = grouped[name]
        dynamic = name in DYNAMIC

        # Static bodies keep the layout's baked world coordinates.  Dynamic ones
        # need a real pivot, so they are recentred on their own base.
        shift = (0.0, 0.0, 0.0)
        pivot_b = (0.0, 0.0, 0.0)
        if dynamic:
            pts = [layout.xyz(v) for fs in by_look.values() for f in fs for v, _, _ in f]
            cx = (min(p[0] for p in pts) + max(p[0] for p in pts)) / 2
            cy = (min(p[1] for p in pts) + max(p[1] for p in pts)) / 2
            cz = min(p[2] for p in pts)
            pivot_b = (cx, cy, cz)
            shift = (-cx, -cy, -cz)

        # An element modelled as an open shell has its inward-facing surfaces
        # culled - exactly where the AUV looks from - so its shell chunks get a
        # mirrored visual copy.  A *fraction* rather than "any open edge at all",
        # because closing a mesh by hand leaves a few stragglers: the pipe run is
        # ~1% open at junctions buried inside fittings and must not be doubled,
        # while a genuinely open shell like the table frame is ~40% open.
        open_fraction = element_boundary(layout, by_look)
        element_open = open_fraction > OPEN_SHELL_FRACTION
        chunks = []
        for look, faces in by_look.items():
            ch = Chunk(layout, faces, shift)
            stem = f"{name}__{look.removeprefix('robo_')}"
            if dynamic and ch.is_flat():
                ch.extrude_flat(ICON_THICKNESS)
            elif look.startswith("robo_icon_") and ch.is_flat():
                ch.nudge_along_normal(ICON_NUDGE)
            phys = f"{stem}.obj"
            ch.write(DATA / phys, f"{name} / look '{look}' - from {LAYOUT}")
            vis = phys
            if element_open and not ch.is_closed():
                vis = f"{stem}_ds.obj"
                ch.double_sided().write(
                    DATA / vis, f"{name} / look '{look}' - from {LAYOUT}")
            chunks.append((phys, vis, look))

        pivot = world_pivot(*pivot_b)
        if name == "octagon":
            pivot = (pivot[0], pivot[1], pivot[2] - OCTAGON_SURFACE_LIFT)
        bodies.append(body_xml(name, chunks, pivot, dynamic))
        ds = sum(1 for c in chunks if c[0] != c[1])
        print(f"{name:22s} {len(chunks)} chunk(s)"
              f"{f', {ds} double-sided' if ds else '':<20s}"
              f"  open {open_fraction * 100:5.2f}%"
              f"  pivot=({pivot[0]:7.3f} {pivot[1]:7.3f} {pivot[2]:6.3f})")

    header = (
        '<?xml version="1.0"?>\n<scenario>\n\n'
        "  <!-- GENERATED by tools/import_robosub_course.py - do not edit by hand.\n"
        f"       Poses come from the Blender export's {LAYOUT},\n"
        "       which is the authority for the layout (the placement table in\n"
        "       STONEFISH_SETUP.md disagrees with it).  Every element's pose is\n"
        "       baked into its vertices, so all bodies share one world_transform.\n"
        "       Frame: X fwd, Y right, Z down; water surface Z=0, pool floor Z=3.432.\n"
        "       Each element is split into one body per material because Stonefish\n"
        "       binds a single <look> per body. -->\n\n"
    )
    (PKG / "objects" / "robosub_course.scn").write_text(
        header + "\n".join(bodies) + "\n</scenario>\n"
    )
    (PKG / "metadata" / "robosub_looks.scn").write_text(
        '<?xml version="1.0"?>\n<scenario>\n    <looks>\n'
        "        <!-- GENERATED by tools/import_robosub_course.py -->\n"
        + "\n".join(looks.defs)
        + "\n        </looks>\n</scenario>\n"
    )
    print(f"\n{len(looks.defs)} looks -> metadata/robosub_looks.scn")
    return 0


if __name__ == "__main__":
    sys.exit(main())
