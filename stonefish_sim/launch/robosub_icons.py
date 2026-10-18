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


# Landmark subtype (vortex_msgs/LandmarkSubtype) shown by each role image, for the
# $(param robosub_landmark_<slot>) subtypes in objects/robosub_landmarks.scn.
ROLE_LANDMARK_SUBTYPE = {
    "Task1_SearchRescue.png": "GATE_SEARCH_RESCUE",
    "Task1_SurveyRepair.png": "GATE_SURVEY_REPAIR",
    "Task3_Blood.png": "BIN_SEARCH_RESCUE",
    "Task3_Fire.png": "BIN_SURVEY_REPAIR",
    "Task5_Repair.png": "OCTAGON_IMAGE_REPAIR",
    "Task5_Rescue.png": "OCTAGON_IMAGE_RESCUE",
    "Task5_Search.png": "OCTAGON_IMAGE_SEARCH",
    "Task5_Survey.png": "OCTAGON_IMAGE_SURVEY",
    "Task5_RedCross.png": "TABLE_BASKET_SEARCH_RESCUE",
    "Task5_Warning.png": "TABLE_BASKET_SURVEY_REPAIR",
    "Task5_BandAid.png": "TABLE_ITEM_BANDAID",
    "Task5_Electric.png": "TABLE_ITEM_ELECTRIC",
    "Task5_NutBolt.png": "TABLE_ITEM_NUTBOLT",
    "Task5_Pill.png": "TABLE_ITEM_PILL",
}

# Torpedo icon centres per board version, in the frame of torpedo_board__icon_torpedo_board
# ($(param robosub_landmark_torpedo_<icon>_point) in objects/robosub_landmarks.scn).
TORPEDO_ICON_POINTS = {
    "Task4_ver1.png": {
        "fire": "-3.8400 13.0430 1.0920",
        "firetruck": "-3.4480 13.0430 1.0710",
        "blood": "-3.4150 13.0430 0.8200",
        "ambulance": "-3.8160 13.0430 0.6760",
    },
    "Task4_ver2.png": {
        "blood": "-3.8440 13.0430 1.1000",
        "ambulance": "-3.4560 13.0430 1.0820",
        "fire": "-3.4210 13.0430 0.8220",
        "firetruck": "-3.8170 13.0430 0.6950",
    },
}


def icon_parameters(seed=None):
    """Return {parameter name: texture path or landmark value} for one run.

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
            if tex in ROLE_LANDMARK_SUBTYPE:
                params[f"robosub_landmark_{slot}"] = ROLE_LANDMARK_SUBTYPE[tex]
            if tex in TORPEDO_ICON_POINTS:
                for icon, point in TORPEDO_ICON_POINTS[tex].items():
                    params[f"robosub_landmark_torpedo_{icon}_point"] = point
            summary.append(f"    {slot:24s} {tex}")
        summary.append("")

    print(f"[robosub] icon seed {seed} "
          f"(reproduce with robosub_icon_seed:={seed})")
    print("\n".join(summary), end="")
    return params
