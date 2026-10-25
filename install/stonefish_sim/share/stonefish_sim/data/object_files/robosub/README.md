# RoboSub venue geometry

Pool basin and venue decor for `scenarios/robosub.scn`, used via
`objects/robosub_venue.scn`.

The **course elements** no longer live here — they were replaced by the Blender
export and now live in `../robosub_course/` (generated; see its README).

## Provenance

Extracted from the MathWorks **RoboSub Virtual Environment** (Unreal Engine 5.3,
`AutoVrtlEnv/Content/Blender/*` and `Content/Maps/RoboSub_pool.umap`).

Pipeline:
1. `umodel` (UE Viewer) → `.pskx` static meshes + `.tga` textures
2. `tools/pskx_to_obj.py` (in this repo's sibling working tree, not packaged):
   `.pskx` → `.obj`, `.tga` → `.png`. Applies scale 0.01 (UE cm → m) and negates
   Z so the meshes drop straight into Stonefish's X‑fwd / Y‑right / **Z‑down** frame
   with `rpy="0 0 0"`.

`meshes.json` holds each mesh's bounding box / centre / size. It still describes
the full original extraction, including the course meshes that were removed.

## Meshes

| file | element | size (m) |
|---|---|---|
| `Pool_PoolPlane` | pool basin (floor at Z≈3.432) | 100 × 75 × 3.43 |
| `Pool_Staircase1/2`, `Pool_BillBoard1/2` | venue decor | — |

The decor meshes carry their own world offsets, so `robosub_venue.scn` places
them at the world origin with `rpy="0 0 0"`.
