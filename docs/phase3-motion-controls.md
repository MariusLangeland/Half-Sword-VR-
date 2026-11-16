# Phase 3: motion controls (the controllers move the arms)

Status 2026-11-15: first version built and installed on Matthew's PC; not yet tried in the headset.
Background: `docs/phase2-arm-research.md`.

## How it works

Half Sword animates an invisible driver skeleton (`DriverSkeleton`, anim `ABP_Willie`). The visible,
physically simulated body (`CharacterMesh0`, anim `ABP_CopyPose1`) copies the driver's bone transforms
each frame in its `CopyPoseFromMesh` node, and the body's muscles pull the real arm bodies towards that
pose. The mod bends the driver's arm bones between those two steps, so the game's strength, stamina,
injuries and weapon weight still decide how well the real arm follows.

1. **Controllers** (`vr_present.cpp`): an OpenXR action set with a grip pose, trigger and squeeze per
   hand. Bindings are suggested for the Index, Touch, Vive and simple controller profiles; SteamVR maps
   the Steam Frame controllers onto one of them. Poses are located at the same predicted time as the
   eyes. The log names the profile in use ("controller uses the bindings for ...").
2. **Into the world** (`stereo_device.cpp`, `stereo_tracking_to_world`): exactly the mapping used for the
   eyes (recenter, head anchor, heading, world scale), so the hands sit where the player sees them.
3. **Hand target** (`arm_control.cpp`, `compute_targets`): the controller's handle direction is matched
   to the direction of the handle in the character's hand, and the controller's "wrist to fist" direction
   to the hand bone's "wrist to grip" direction. Both are measured on the physical body a few times a
   second: from the weapon in the right hand (`Weapon R` actor location to its `Head` component), or,
   with no weapon, from the finger bones (`middle_01`, `thumb_01`). So the blade points where the
   controller's handle points.
4. **Hook point**: every other skeletal mesh of the character gets its `PrimaryComponentTick`'s
   vtable pointer replaced by a copy whose `ExecuteTick` first runs the arm solver (per object; other
   objects of the type are untouched). `FTickFunction` has four virtuals in 5.4.4: destructor,
   `ExecuteTick`, `DiagnosticMessage`, `DiagnosticContext`. The first mesh to update after the driver
   has animated applies the arms; later ones in the same frame see it done.
5. **Driver bone transforms**: `USkinnedMeshComponent::ComponentSpaceTransformsArray[2]` followed by two
   arrays, a float, `CurrentEditableComponentTransforms` and `CurrentReadComponentTransforms` (+68 and +72
   from the arrays). Not reflected, so found by shape and confirmed against `GetSocketTransform` in
   component space for both hands and upper arms. The read buffer is the one `CopyPoseFromMesh` reads.
6. **Solver** (`solve_side`): the hand target is taken relative to the physical upper body (the
   clavicle's parent bone) and placed on the driver's upper body. Two-bone IK from the animated shoulder:
   elbow towards a pole (down, out, back of the character's facing; `elbow_*`), the elbow bends about its
   own hinge axis, which is learned from the game's own animation (logged once per arm). The forearm takes
   `forearm_twist_share` of the hand's roll. Bones below the hand (fingers, `weapon_r`) are carried along.
   Out-of-reach targets are clamped to 99.9 % of the arm length.
7. **Aim** (`sync_aim`): the pawn's `Aim Vector` (the 2-D aim the mouse normally moves, ±90) is set from
   the blade direction relative to the character's facing, so the game's swing speed, stamina and body
   turning follow along (`aim_sync`).

If any check fails (layout not found, unexpected parameter sizes, arm chain not as expected) the log says
so and motion arms stay off; the game runs as before.

## Use

- Controllers must be on and tracked. Hold the sword controller like a sword handle.
- **Page Up** switches motion arms on and off while playing (the game's own arm animation comes back).
- Home recenters as before; recenter while standing as you will play.
- Settings in `HalfSwordVR.ini` (defaults apply when a key is missing):

| Key | Default | Meaning |
| --- | --- | --- |
| `motion_arms` | 1 | Controllers move the hands |
| `arm_right`, `arm_left` | 1 | Per arm |
| `toggle_arms_key` | 0x21 | Page Up |
| `aim_sync` | 1 | Game's aim follows the sword |
| `aim_scale`, `aim_invert_y` | 1, 0 | Aim mapping |
| `elbow_down`, `elbow_out`, `elbow_back` | 1, 0.6, 0.2 | Where the elbow points |
| `forearm_twist_share` | 0.5 | Hand roll taken by the forearm |
| `grip_shift_cm` | 0 | Slide the sword along the handle |
| `hand_pitch`, `hand_yaw`, `hand_roll` | 0 | Extra turn of the hands |
| `log_arms` | 1 | "Arms:" status line once a second |

Developer switches: `dev_fake_hands` (pretend controllers circling in front of the chest),
`dev_arms_any_character` (drive the first `Willie_BP_C` in the level when nobody plays one).

## Log lines to read after a test

- `Arms: driver skeleton bone transforms found at offset ...`, `Arms: ready on ...`: setup worked.
- `Arms: ... elbow hinge learned ...`, `Arms: ... hand grip measured from the weapon in it / its fingers`.
- `Arms: on | controllers L tracked R tracked | right hand sent err X cm Y deg | ... driver frames fresh
  N late M applied K`: err is the distance between where the physical hand is and where it was sent
  (the muscles' lag and weakness). "late" counts frames where the body updated before the driver had
  animated (the previous pose was reused), which adds one frame of delay.

## Unattended test notes

- `open /Game/Maps/Arenas/Map_Arena_Cellar` (and Slums) from the splash screen loaded the arena but
  spawned no character on 2026-11-15, so the arm code could not be exercised without someone playing.

## Hold assist raised (2026-11-16)

The first fight log showed the right hand sitting 15 to 25 cm and 30 to 50 degrees behind what was
sent, while the hold assist was capped at `assist_max_cm 15` and `assist_max_deg 35`, so its
correction was saturated. The caps are now 25 cm and 45 degrees; `assist_seconds` stays 0.4, so
fast swings are still not changed by it. Both keys are read from `HalfSwordVR.ini` when present.

## Driver pose solver (FABRIK)

The hands were first placed by solving each arm with a two-bone IK, and an experiment replaced that
with a general "place this bone here" primitive. That experiment failed for a specific reason worth
recording: the visible body is a physical ragdoll and its muscles are **angular drives**. A pose
whose bone positions were moved without preserving the distances between joints cannot be
reproduced by the ragdoll, so the hand followed only its rotation and the forearm/upper arm visibly
came apart.

The solver therefore works in rotations with **fixed segment lengths**:

- **FABRIK** over the chain `hand ← forearm ← upper arm ← clavicle ← spine_05` (`pose_reach_bones`,
  default 5; 4 = arm only). Segment lengths are taken from the animated pose and preserved, so the
  result is a pose the ragdoll can follow, position and rotation both.
- **Soft reach limit**: exact up to `pose_reach_knee` (0.88 of arm length); beyond that the request
  is compressed exponentially toward `pose_reach_max` (0.97) and time-smoothed
  (`pose_reach_smoothing`, 0.12 s). The elbow never locks straight and there is no snap.
- **Torso help**: bones above the clavicle take `pose_reach_spine_share` (0.35) of their solved
  rotation; the share rises toward 1 as the target exceeds reach, so the chest leans in instead of
  the hand simply dragging behind.
- **Arm scale**: `pose_arm_scale` (1.0) scales the requested distance, for players whose arms are
  longer than the character's.
- **Hand rotation** comes from the controller (`pose_hand_rotation`).
- The old two-bone IK path is still selectable with `pose_system=0`.

Everything input related was removed: there is no walking, turning, jumping or button handling in
the mod any more; only the driver skeleton is driven. The arbitrary-bone placement API of the
failed experiment was removed with it.

Debug view: `debug_draw=1` makes the game's hidden `DriverSkeleton` mesh visible, which shows the
pose the solver produces. The engine's own `DrawDebugLine`/`DrawDebugSphere` are compiled out of
Shipping builds, and drawing into the runtime's swapchain from this game's Present path crashed the
presenter twice, so neither is used.

## Head as a pose target (not enabled)

Placing the Head bone at the headset position was implemented (`pose_head`, `pose_head_bones`) but
is **off by default**. With the camera anchored to the character's head bone the head target fed
back into the view: offsetting the head made the character bend and drift. A body-anchor camera
(`anchor=body`, actor root + measured eye offset) exists to break that loop but has not been
verified yet.

## Open


- The FABRIK solver still leaves a small steady offset from the controller position, and the hand
  falls further behind when the player's arms are longer than the character's (tune
  `pose_arm_scale`, `grip_shift_cm`, `hand_pitch`/`yaw`/`roll`).
- The head target is implemented but disabled: the body-anchor camera must be verified first.
- Two-handed grip now puts the left hand on the hilt, but it is a fixed offset and has not been
  tuned against real combat.
- Physics assist for tip accuracy (option C in the research notes) only if the muscles prove too weak.
