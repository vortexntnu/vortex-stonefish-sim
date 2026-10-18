# RoboSub 2026 course meshes (generated)

**Everything in this directory except this README is generated — do not edit by
hand.** Regenerate with:

```bash
tools/import_robosub_course.py <path-to>/Stonefish_Export \
  --extra-textures "<path-to>/Task Design (Provided by RoboNation)/Task 04 _ Deploy (Torpedoes)/Official images"
```

The `--extra-textures` folder is optional but recommended: the handbook says two
versions of the torpedo board exist, and the Stonefish export ships only
`Task4_ver2.png`. `Task4_ver1.png` lives in the RoboNation task-design tree, and
without it the board can only show version 2 (the importer says so and carries on).

## Source: the layout OBJ, not the placement table

The importer reads **`robosub_course_layout_COMBINED.obj`** — the whole course
fused at its final laid-out positions. That file, not the placement table in
`STONEFISH_SETUP.md`, is the authority for where things go.

The two disagree, and the layout wins:

| element | placement table | actual layout |
|---|---|---|
| `torpedo_board` | (-4.5, 20.0), yaw 180° | (-2.323, 15.222), yaw 0° |
| `pathmarker_1` | (0.0, 2.2) | (≈1.96, ≈3.31) |
| `pathmarker_2` | (2.257, 12.473) | (2.257, ≈11.86) |
| `bins_pipeline` | yaw −63.64°, flat | tilted — not expressible as yaw-only |

The per-element OBJs are **not** used either: their geometry has drifted from
the layout. The standalone `bins_pipeline.obj` bins are 0.34 × 0.34 × 0.279 and
axis-aligned; the laid-out ones are 0.471 × 0.502 × 0.352 and tilted. Reading
the layout directly makes all of this moot — the fused file contains exactly the
same 36 131 vertices as the per-element files summed, so nothing is lost.

## What the importer does

1. **Groups layout objects into elements.** `mesh*` → `bins_pipeline`,
   `octagon*` → `octagon`, `restore_table`/`basket_*` → `restore_table`, and so
   on (see `element_of`). Each `bin_*` is its own element — see icon slots below.
2. **Splits each element by material.** Stonefish binds exactly one `<look>` per
   body, so a multi-material element cannot be one body. Faces are grouped by
   *resolved look* (not raw material name) so the several Blender materials that
   differ only in an unused MTL field collapse into one chunk. Output is
   `<element>__<look>.obj`.
3. **Converts frames — once, globally.** Because the layout already bakes every
   element's pose into its vertices, a single transform applies to everything:

   | | |
   |---|---|
   | `X_w` | `Y_b + 4.0` |
   | `Y_w` | `X_b - 1.57` |
   | `Z_w` | `3.432 - Z_b` |

   which is `world_transform xyz="4.0 -1.57 3.432" rpy="π 0 π/2"` on every body
   (`Rz(90°)·Rx(180°)`). The `(4.0, −1.57)` offset puts the gate 4 m in front of
   the default drone spawn with its opening centred on Y=0. Dynamic pickups are
   the exception: they get recentred on their own base so they have a real pivot.
4. **Double-sides open shells.** Stonefish backface-culls. The bins and the
   table's baskets are modelled as open shells with outward-facing normals, so
   their walls and floor icons vanish when the camera is *inside* them — which
   is exactly where the AUV looks from. Chunks of an element that has a genuine
   boundary edge get a mirrored-face `_ds.obj` copy used for `<visual>`;
   `<physical>` keeps the original. Chunks that are closed solids on their own
   are skipped, and closed elements (gate, octagon, torpedo board, pathmarkers,
   slalom, pickups) are untouched.
5. **Solidifies flat decals inside dynamic bodies.** The Task 5 pickup icons are
   zero-thickness discs. As parts of a compound rigid body they give zero mass
   and NaN inertia, and Bullet drops the body with *"Overflow in AABB, object
   removed from simulation"*. The importer extrudes them 2 mm, stitching the
   boundary edges and emitting flat per-face normals — Stonefish's OBJ reader
   indexes `normals[nID-1]` unconditionally, so a mesh with no `vn` lines
   segfaults it.
6. **Emits looks** into `metadata/robosub_looks.scn`, deduplicated across
   elements by texture path or by `Kd` colour — except role images, which become
   per-slot looks (below).
7. **Creates icon slots.** Every role image gets its own uniquely named look
   (`robo_icon_<slot>`), so the image a slot shows can be changed in
   `metadata/robosub_looks.scn` without touching the meshes. This is why
   `bin_1`…`bin_4` are separate elements rather than part of `bins_pipeline`.

## Generated outputs

| path | |
|---|---|
| `<element>__<look>.obj` | per-material mesh chunks (collision + visual) |
| `<element>__<look>_ds.obj` | double-sided visual copy, where needed |
| `textures/*.png` | every PNG referenced by the MTLs |
| `../../../objects/robosub_course.scn` | the course bodies |
| `../../../metadata/robosub_looks.scn` | the looks |

## Resulting world placement

Depth is Z (down-positive, water surface 0, pool floor 3.432).

| element | X | Y | Z |
|---|---|---|---|
| gate | 4.00 | -0.02 | 2.75 |
| pathmarker_1 | 7.31 | 0.39 | 3.35 |
| slalom | 12.89 | 0.41 | 2.79 |
| pathmarker_2 | 15.86 | 0.69 | 3.35 |
| torpedo_board | 19.00 | -3.57 | 2.83 |
| bins_pipeline | 19.47 | 4.27 | 2.94 |
| octagon | 22.25 | 0.11 | 0.20 |
| restore_table | 22.25 | 0.11 | 3.05 |
| restore_jar_1 / _2 | 22.13 | -0.01 / 0.23 | 2.63 |
| restore_container_1 / _2 | 22.37 | -0.01 / 0.23 | 2.64 |

## Role images

The course is fixed: no seed and no randomization. Each slot's image is set in
`metadata/robosub_looks.scn`, and `objects/robosub_landmarks.scn` gives the
landmark subtype each slot shows (they must agree when you change an image):

| slot | image |
|---|---|
| gate_1 / gate_2 | Search & Rescue / Survey & Repair |
| bin_1, bin_2 / bin_3, bin_4 | Blood / Fire |
| torpedo_board | version 1 |
| octagon_1 … _4 | Repair, Rescue, Search, Survey |
| restore_table_1 / _2 | Red Cross / Warning |
| restore_jar_1, _2, restore_container_1, _2 | Band-Aid, Electric, Nut & Bolt, Pill |

### Other valid layouts (if you change images by hand)

The handbook fixes *which* images exist and *how many* of each, and leaves only
their placement open. When changing images, keep each group a **permutation of
its pool** — exactly one Survey & Repair gate side, exactly two 🔥 bins and two
🩸 bins, exactly one of each octagon plate.

| group | handbook | slots | arrangements | rule |
|---|---|---|---|---|
| `gate_roles` | §3.2.2 | 2 | 2 | "Survey & Repair on one side (**could be either side**), Search & Rescue on the other" |
| `bin_roles` | §3.2.4 | 4 | 6 | "four bins. **Two** for Survey & Repair and **two** for Search & Rescue" |
| `torpedo_board` | §3.2.5 | 1 | 2 | "There are **two different versions** of the board" |
| `octagon_plates` | §3.2.6 | 4 | 24 | "hanging from the octagon, facing inward, are images of 🧭 ⚒️ 🛟 🆘" — one each, positions unspecified |
| `basket_floors` | §3.2.6 | 2 | 2 | "⚠️ at the bottom of the Survey & Repair basket, ⛑️ at the bottom of Search & Rescue" — which edge is unspecified |
| `table_items` | §3.2.6 | 4 | 24 | "two different items for each role: 🔩 and 🔌; 💊 and 🩹" — one each, table positions unspecified |

**Never change these**, because the handbook pins them down as
navigation cues:

- The gate's corrugated panel pattern — **RED upper-right / BLACK lower-right**,
  BLACK upper-left / RED lower-left ("Red, Right, Above", §3.2.2). Changing
  this would break gate-side identification.
- The slalom's **White / Red / White** pipe colours (§3.2.3).
- The red gate divider.
- Element **positions** — placement and texture choice are fully independent, and
  the layout comes from the Blender file.

### Not modelled

Two per-run variables in the handbook have no geometry yet, so nothing here
covers them:

- **Task 3 magnetic lights** (§3.2.4): "at the start of a run, the light will be
  turned on signaling the need for repair or rescue". Which bins are lit is a
  per-run variable for when the lights exist.
- **Task 1 coin flips** (§3.2.1): starting heading, and role assignment when no
  visual indicator is available. These affect the *vehicle*, not the course.

## Caveats

- **`octagon` Z.** The layout models it 1.8 m up, which the guide flags as a
  placeholder for "floats near the surface". The importer lifts it to the actual
  water surface for this pool depth (`OCTAGON_SURFACE_LIFT`). This is the one
  place the importer deliberately departs from the layout file.
- **Pickup buoyancy.** The four Task 5 items use the `RoboPickup` material
  (ρ = 1150) so they rest on the table instead of floating off it.
- **Pipe run size.** `bins_pipeline__white` is 45 315 triangles and, being an
  open shell (the pipe segments are uncapped at their butt joints), its
  double-sided visual copy is 90 630. If this ever becomes a rendering cost,
  that chunk is the one to look at.
