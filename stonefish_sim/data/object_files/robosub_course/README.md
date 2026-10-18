# RoboSub 2026 course meshes (exported from Unity)

The pool and course of the Unity simulator's `RoboSub2026.unity` scene
(`auv-sim-unity`), exported with **Tools → Vortex → Export Course to Stonefish**:

| path | |
|---|---|
| `<prop>__<material>.obj` | one mesh per course element and material (collision + visual) |
| `textures/*.png` | the role images and other textures |
| `../../../objects/robosub_course.scn` | the bodies |
| `../../../metadata/robosub_looks.scn` | the looks |
| `../../../objects/robosub_landmarks.scn` | the landmarks, from the Unity `VortexLandmark` components |

The vertices are already in the world frame, which is the Vortex odom frame:
origin at Orca's start position on the surface, X down the course, Y right,
Z down (depth). So every body has an identity `world_transform`, and the
course is fixed: no seed and no randomization.

## Editing

- **In the simulator:** press **E**, click an element, drag it (Shift: up/down),
  move or add its landmarks, then **Save**. This rewrites the `world_transform`
  of the element's bodies and the tags in `robosub_landmarks.scn`.
- **In Unity:** move things in `RoboSub2026.unity` and export again. This
  replaces everything listed above, including edits made in the simulator.

The parts of one element share the name before `__` (`gate__redmat`,
`gate__whitemat`, ...), which is how the simulator's editor moves them together.

## Re-exporting

```bash
~/Unity/Hub/Editor/6000.6.6f1/Editor/Unity -batchmode -nographics -quit \
  -projectPath ~/auv-sim-unity -executeMethod StonefishExporter.ExportBatch \
  -stonefishSim ~/ros2_ws/src/vortex-stonefish-sim/stonefish_sim
```
