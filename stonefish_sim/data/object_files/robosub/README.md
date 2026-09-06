# RoboSub course geometry

3D meshes for the RoboSub competition course, used by `scenarios/robosub.scn`
via `objects/robosub_course.scn`.

## Provenance

Extracted from the MathWorks **RoboSub Virtual Environment** (Unreal Engine 5.3,
`AutoVrtlEnv/Content/Blender/*` and `Content/Maps/RoboSub_pool.umap`).

Pipeline:
1. `umodel` (UE Viewer) → `.pskx` static meshes + `.tga` textures
2. `tools/pskx_to_obj.py` (in this repo's sibling working tree, not packaged):
   `.pskx` → `.obj`, `.tga` → `.png`. Applies scale 0.01 (UE cm → m) and negates
   Z so the meshes drop straight into Stonefish's X‑fwd / Y‑right / **Z‑down** frame
   with `rpy="0 0 0"`.
3. `tools/gen_robosub_scn.py` → `objects/robosub_course.scn`.

`meshes.json` holds each mesh's bounding box / centre / size (used by the generator
to centre task elements on their body frame).

## Meshes

| file | element | size (m) |
|---|---|---|
| `Pool_PoolPlane` | pool basin (floor at Z≈3.43) | 100 × 75 × 3.4 |
| `Pool_Staircase1/2`, `Pool_BillBoard1/2` | venue decor (carry their own world offsets) | — |
| `gate` | Task 1 gate | 3.15 × 1.55 |
| `pathmarker` | orange path marker | 0.61 long |
| `slalom_spaced` | Task 2 slalom pipe sets | 4.1 × 4.1 |
| `robosub_box` | Task 3 bins / pipeline | 0.85 × 0.85 × 1.14 |
| `SM_torpedo26` | Task 4 torpedo board | 0.72 × 1.25 |
| `task1_repair`, `task1_rescue` | role image plates | 0.30 × 0.30 |
| `octagon` | Task 5 octagon surface marker | 2.54 dia |

## Layout — inspection grid (current)

`robosub_course.scn` currently parks every mesh in a **compact grid ~4-17 m in
front of the drone spawn**, all centred at depth Z = 1.4, so each can be checked
for scale / orientation / material. This is *not* the competition layout - once
the meshes look right it gets spread out per the handbook.

Regenerate with `tools/gen_robosub_scn.py`.

## Competition layout — approximate

Placement in `robosub_course.scn` follows the **RoboSub 2026 Team Handbook §3.2**
(Start → Gate → Slalom → Bins → Torpedoes → Octagon, staggered so no three
elements are collinear). The handbook does **not** publish exact course
coordinates, and `umodel` cannot export the `.umap` actor transforms, so the
positions are a sensible reconstruction — expect to nudge them.

To get the **exact** UE layout, a `Mappings.usmap` from the running game is needed;
then `tools/ripper` (CUE4Parse) dumps `level.json` with every real transform.
See `~/code/tools/README_EXTRACT.md`.
