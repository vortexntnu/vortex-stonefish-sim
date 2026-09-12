"""Randomized role images for the RoboSub course, chosen at launch.

The generated looks in `metadata/robosub_looks.scn` declare their textures as
`$(param robosub_icon_<slot>)`, which stonefish_ros2 substitutes from the
simulator node's parameters.  This module produces those parameters.

What may vary and what may not comes straight from the RoboSub 2026 Team
Handbook section 3.2 — see `metadata/robosub_icons.json`, which the importer
generates alongside the meshes.  In short: the handbook fixes *which* images
exist and *how many* of each, and leaves only their placement open, so every
group is a permutation of a fixed pool rather than a free draw.  Nothing here
touches the gate's Red/Black panel pattern or the slalom's pipe colours, which
the handbook specifies as navigation cues.

torpedo_board is included as a "choice" group (2.5): its decal is modelled on
both faces of the board (see --element-obj in tools/import_robosub_course.py),
so which of the two handbook versions shows is safe to randomize regardless of
which way the board actually faces.
"""

import json
import os
import random

from ament_index_python.packages import get_package_share_directory


def icon_parameters(seed=None):
    """Return {parameter name: texture path} for one run.

    `seed` may be an int, a string, or None/"" for a fresh random draw.  Pass a
    seed to reproduce a previous course layout exactly.
    """
    share = get_package_share_directory("stonefish_sim")
    with open(os.path.join(share, "metadata", "robosub_icons.json")) as f:
        manifest = json.load(f)

    if seed in (None, ""):
        seed = random.randrange(2**31)
    else:
        try:
            seed = int(seed)
        except (TypeError, ValueError):
            pass
    rng = random.Random(seed)

    tex_dir = manifest["texture_dir"]
    params = {}
    summary = []
    for group in manifest["groups"]:
        slots = group["slots"]
        pool = list(group["pool"])
        if not pool:
            continue
        if group["mode"] == "choice":
            picks = [rng.choice(pool) for _ in slots]
        else:
            rng.shuffle(pool)
            picks = pool
        for slot, tex in zip(slots, picks):
            params[f"robosub_icon_{slot}"] = f"{tex_dir}/{tex}"
            summary.append(f"    {slot:24s} {tex}")
        summary.append("")

    print(f"[robosub] icon seed {seed} "
          f"(reproduce with robosub_icon_seed:={seed})")
    print("\n".join(summary), end="")
    return params
