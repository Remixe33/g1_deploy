# Source provenance

Parkour inference is additionally adapted from the local checkout of
`https://github.com/wangsy1999/unitree_mujoco`: `example/cpp/g1_parkour_policy.h`,
`example/parkour/g1.json`, and `exported/g1_parkour/`, at commit
`7c6642f89e25f72dd7de3cf7683f166bce943fef`.
The original BSD project license is retained. Network/config details and SHA256
values are recorded in [docs/PARKOUR.md](docs/PARKOUR.md).
No MuJoCo simulator or mesh assets are copied into this repository; the only XML
asset is the door scene copied from the local OpenDoor-GraspPlan checkout (below).

Extracted from the working tree of `wangsy1999/unitree_mujoco`, based on commit `7c6642f89e25f72dd7de3cf7683f166bce943fef`.
The source includes subsequent local DreamWaQ hardware deployment changes.
This is an independent source tree; no submodule, symlink or parent repository is required.
The original repository LICENSE is retained. Unitree SDK2 and ONNX Runtime are external dependencies.

The web monitor was synchronized from the tested robot deployment on 2026-10-19.
It originates from `simulate_python/joint_monitor.py` and
`simulate_python/web/joint_monitor.html` in the same `unitree_mujoco` commit above,
with the onboard domain-0, dry-run telemetry and Chinese UI changes retained.
In this repository it lives in `apps/monitor/joint_monitor.py` and
`apps/monitor/static/joint_monitor.html`; only the static-resource path was adjusted.
`apps/monitor/monitor.sh` replaces the robot-specific absolute-path launcher with a
relocatable source/install launcher. Python SDK and CycloneDDS are external dependencies.

Model: `apps/dwaq/models/policy.onnx`
SHA256: `9b0675da43e047084fac72fab9221d54c5306e9f7f4ccd36e651a61841d2d13e`

Configuration: `apps/dwaq/config/g1_locomotion.json`
Original SHA256: `09f7c133dc5e043aed7572d545e51f12dadf59cd11bcafa338e452c890db0c59`
Current SHA256: `568731c0f88d0ad40aa9fc3dc81b9389b0c6b07244f65d7ce29c952183b00a81`
The current model is copied from
`/home/wsy/rl/lab3/logs/rsl_rl/g1_dwaq/2026-11-13_10-08-23/exported/policy.onnx`.
Deployment parameters are aligned with this run's `params/env.yaml` and `params/agent.yaml`:
five whole observation frames ordered oldest to newest (`history_order: time`),
waist roll/pitch stiffness 50, raw action clipping at 100 with clipped action feedback,
and no policy joint-target clipping. Existing hardware target limits remain active.
The 20 ms period, joint mapping, default pose, action scale, observation scales
and command ranges match the training run.

The DreamWaQ deployment configuration is serialized as JSON.
Parkour maintains its own `hardware` section in `apps/parkour/config/g1_parkour.json`; hardware settings are no longer shared between programs.

Model paths are configured in each program's JSON, relative to the configuration directory.

Door scene asset: `apps/simulation/assets/doors/` (MJCF + door config JSONs), copied verbatim on 2026-11-17
from the local checkout `/home/xxx/Projects/OpenDoor-GraspPlan/assets/doors/` (door generator `shared-door-v2`,
door config `doorman_right_out_no_latch.json`, scene layout `doorman_right_out_with_walls.json`).
Primitive geoms only: no meshes, no actuators. Used by `--terrain doors` in `apps/simulation/terrain.py`.
SHA256:
- `shared_doorman_v1.mjcf.xml`: `555b74e4becf3018c788480cea3c4bcc328fc683dd520f2a5c6be567ba1d5d87`
- `doorman_right_out_no_latch.json`: `750ebb364b717ea08a006e36fda54d19d049c95c505f4a03a65609bac5753004`
- `doorman_right_out_with_walls.json`: `f36efcb496a38cdc8163ae2802338b02f658ff7cf9c8b515e11fe4038afb10e8`

Optional head-back mounting recipe: `apps/simulation/terrain.py` (`_apply_head_back`, `apply_head_back_camera`)
transcribes the static head mount from the local OpenDoor-GraspPlan branch `codex/g1-head-pitch20`
(commit `7200116`, `src/opendoor_plan/hierarchical/head_pose.py`), selected via `terrain.json` `"head": {"back_deg": ...}`.
Whole head_link is rotated about torso +Y by -back_deg around torso-frame pivot [0, 0, 0.320] m
(user-specified 2026-11-13, not calibrated); the head contribution (mass 1.036 kg, COM and inertia read from
`ext/unitree_ros/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf`, `head_joint` + `head_link`
`<inertial>`, head COM in torso frame [0.0092305, 0.000299, 0.405869]) is merged back into the torso `<inertial>`
with total mass kept constant. The depth camera follows the same pivot transform (mount point through H,
optical axis re-pitched) in `apps/simulation/sim.py`'s publisher, mirroring OpenDoor's `realsense.py` mount mapping.
No asset files are copied for this feature; only the recipe and the constants above are transcribed.
