# Teleoperation experiments (2026-10-02)

Features proposed from the operator's seat, implemented behind toggles so each can be kept or
deleted after a headset test. Every toggle lives in the **Experiments** panel next to the HUD bar
(PlayerPrefs `Exp/<key>`), each feature in its own script under `Assets/Scripts/Experiments/`, so
deleting one = delete the file + its line in `ExperimentSettings`. Pictures of each feature are in
`docs/experiments/shots/` (Editor harness against the live sim, plus on-device frames).

Operator pain points behind the list: in first person the bar (and so every status) is hidden;
the hand cameras are gone exactly when grasping; nothing says how far behind the robot is;
nothing warns that the headset is live-controlling the robot; image latency grows silently.

| # | Key | Feature | Why | Default | Test |
|---|-----|---------|-----|---------|------|
| 1 | *(always on)* | **Latest-frame decode**: images are decoded once per frame from the newest message; older queued ones are dropped (counted). | Under a backlog the app decoded the oldest message and skipped newer ones: latency grew. | on | Harness floods a camera topic faster than maxFps, asserts the shown frame is the newest and `DroppedFrames` > 0. |
| 2 | `rtt` | **Round-trip latency**: the headset's own `hmd_odom` TF (wall-clock stamp) comes back from the endpoint; RTT = now − stamp, shown as ms. | Tells the operator how stale everything is. | on | Harness turns control on against the live endpoint and asserts an RTT sample 1–500 ms. |
| 3 | `status` | **Status strip** in first person: connection, CONTROL ON (red) / layout, image fps + age, TF Hz, RTT, head pan/tilt gauge (tilt up-positive), lift height bar. Small, head-locked, bottom of view. | The bar is hidden in first person; the head is hidden so its direction is unknown. | on | Screenshot; values match model/ImageSubscriber. |
| 4 | `handcams` | **Hand cameras in first person** (v2): PiP cards outboard and low of each gripper (`hand_*_end_effector_link`): 0.28 m to the side, perpendicular to the line of sight (`normalize(cross(up, hand - head))`, on the side the hand is in head space), 0.05 m below the hand, facing the head, so they no longer cover the head-camera image; option "corners" = head-locked bottom corners. The head-camera image has the same name frame as a camera block (card + label above). | Grasping needs the wrist view; blocks are hidden in first person. | at hands | Screenshot with the sim hand cams live; cards follow the wrists when an arm moves. |
| 5 | `targets` | **Arm target markers**: small spheres at `left_target_link` / `right_target_link` (from `/tf`, parent base frame) + a line to the model's end effector. | Shows tracking error and lag between where I point and where the arm is. | on | Container publishes static target TFs; screenshot shows markers + lines; distance logged. |
| 6 | `basevel` | **Base velocity arrow** (v2) on the floor at the model base from `/sobit_home/odom` twist: filled flat arrow mesh (0.35 m + speed, 0.08 m wide, from 0.35 m ahead of the base), filled 60° turn arc band with arrowhead, floating value chip ("0.15 m/s", "0.50 rad/s"). | Driving blind in first person; shows actual motion incl. lag. | on | Container sends `cmd_vel` for 2 s; arrow appears/scales; screenshot. |
| 7 | ~~`deadman`~~ | ~~Grip to control~~ | removed (user feedback) | — | — |
| 8 | `headlock` | **Image follows my head** (instead of the robot's head frame). *Kept for more testing.* | Hides head lag for comfort; the model still shows the real head. | off | Screenshot: quad centred while the robot head is panned 0.5 rad. |
| 9 | ~~`menurecenter`~~ | ~~Long-press menu = Recenter~~ | removed (redundant with the Meta long-press, which already triggers Recenter) | — | — |
| 10 | *(design)* | Key light on the model; the HUD bar is compact (0.7×, 2.94 m wide) in both modes (v2) and the camera blocks use the space this frees (`minBottom` -1.45 -> -1.75); in first person it also opens 0.35 m lower so it never covers the image centre; status strip v2 with labelled HEAD box (crosshair, dot = pan/tilt, L/R ticks, "pan 29° tilt 17°") and LIFT bar ("0.40 m"); Experiments panel styled like the bar. | Model was near-black; bar covered the image; the gauge was not self-explanatory. | — | Screenshots before/after. |

## Status
Filled in as work lands (agent updates this table): **planned → implemented → tested (harness) → tested (device)**.

| # | Status | Picture |
|---|--------|---------|
| 1 | tested (harness), tested (device) | `shots/01_latest_frame.png` |
| 2 | tested (harness) | `shots/02_rtt_strip.png` |
| 3 | v2 (labelled HEAD box + LIFT bar) tested (harness), tested (device) | `shots/03_status_strip.png`, `shots/11_device_strip.png` |
| 4 | v2 (outboard/low placement + head-camera name frame) tested (harness), tested (device) | `shots/04_handcams_hands.png`, `shots/04_handcams_corners.png`, `shots/02b_fpv_card.png`, `shots/11_device_strip.png` |
| 5 | tested (harness), tested (device) | `shots/05_targets.png`, `shots/12_device_targets.png` |
| 6 | v2 (mesh arrow, arc band, chip) tested (harness), tested (device) | `shots/06_basevel.png`, `shots/13_device_basevel.png` |
| 7 | removed |  — |
| 8 | tested (harness), tested (device) | `shots/08_headlock.png` |
| 9 | removed | — |
| 10 | v2 (0.7 bar in both modes, lowered in first person) tested (harness), tested (device) | `shots/10_bar_compact.png`, `shots/10_experiments_panel.png`, `shots/10_key_light.png` |

Harness = Editor Play mode in a scratch copy of the project against the live SOBIT HOME sim
(v2 round: `ExperimentsVerify` batch 1: 51/51, `ExperimentsVerify2` batches 2–3: 82/82; regressions
LayoutVerifier 107/0, AddRobotTest 38/0, SelectionShot 10/0, FirstPersonVerify 50/50). Device = Quest 3S,
IL2CPP build, 40 s on-device recording (`--ei record 40 --ei fps 10 --ei switchat 5`) with the
experiments at their defaults, while the container moved the head (0.4/0.2), published both arm
targets for 10 s, moved the left arm and sent a linear + angular cmd_vel burst (0.15 m/s, 0.5 rad/s). Experiment toggles can be set at launch for such tests:
`--es exp "headlock=1,status=0"` (`key=0|1`, comma separated; saved like a click in the panel).

## Findings
- **1 Latest frame**: head camera at 25 Hz with maxFps 15 shows 15 fps (badge 14–16), the rest is
  dropped (+51 in 5 s); at maxFps 2 the shown frame was the newest received message 12/12 times,
  interval 0.500 s, +142 drops in 6 s. Decode 1.2–1.4 ms per frame in the Editor, 2.8–4.8 ms on the Quest.
- **2 RTT**: 7–11 ms smoothed (max over 1 s ≈ 41 ms) against the local endpoint, using an injected
  `hmd_odom` TF (the Editor has no XR head, so the publisher itself sends nothing); stops 2.0 s after
  control off. Not measured on the device (v1 run: with the since-removed "Grip to control" on, nothing was published).
- **3 Strip** (v2, 1060 × 96 mm): pan/tilt read back exactly (0.500 / +0.300 rad); the HEAD dot sits at
  (−27, +9) mm, i.e. left for pan +0.5 and up for tilt +0.3; text "pan 29° tilt 17°"; LIFT "0.40 m"
  (fill 32.5 of 56 mm); every text inside the strip. The bold CONTROL ON label shrinks to fit its pill.
  On the device the main row (connected, CONTROL ON, fps, TF, RTT) is legible in the 1280×720 recorder
  frames, the small HEAD/LIFT labels and values are not (to check in the headset itself).
- **4 Hand cams** (v2): 2 cards, frames within 5 s at 15 fps each with the blocks hidden. Placement rule
  holds at the home pose, after `armleft` and after corners → hands: each card 0.28 m from the head→hand
  line (≥ 0.2), left card left of the right one in head space (x −0.49 / +0.43 m), y = hand − 0.05 m
  exactly. With the arms at home the grippers (and so the cards) are 49–64° below the eye line: the
  picture looks down 42° (a 20° look-down does not reach them). Corners mode parents them under the head
  at (±0.52, −0.35, 0.9); off clears their forced decoding (0 hand frames in 2 s).
  Head-camera card: label = the head panel's Label ("Head Camera"), the card encloses the image and the
  label sits above it inside the card.
- **5 Targets**: marker at (∓0.2, 0.9, 0.4) in model-root space to < 1 mm for ROS (0.4, ±0.2, 0.9);
  error = |marker − end effector| (0.24 m at the home pose); hidden 0.7–0.85 s after the publisher stops.
- **6 Base velocity** (v2): arrow and arc are MeshRenderers (no LineRenderer), material URP Simple Lit
  made transparent (queue 3000, alpha 0.75). Linear.x 0.15 → arrow along the base's forward (dot 1.000),
  chip "0.15 m/s"; Angular 0.50 → arc band shown, chip "0.5 rad/s"; both together → "0.13 m/s  0.4 rad/s";
  everything hidden 0.4 s after odom reports the base still. On the Quest the arrow blends exactly like
  in the Editor (arrowhead pixel (42,104,177) vs Editor (41,103,174) over the same floor: translucent, not
  black); the recorder looks level so the arc and chip stay below its view. In the Editor picture the
  arrow shaft runs over the chip (chip anchored at the arrow start). Note: the sim base keeps its last
  cmd_vel, so a test must send a zero Twist.
- **7 Grip to control**: removed after user feedback (v1 findings dropped).
- **8 Image follows my head**: the image hangs at (0, 0, 1.5) under the head and stays centred while the
  model's head pans 28.6°; off puts it back on `head_camera_color_frame`.
- **9 Long-press menu**: removed (the Meta long-press already triggers Recenter; the bar button and the Quest recenter event still do).
- **10 Bar / panel / light** (v2): the bar is 0.7× (scale 0.7/1000) in both modes; in first person
  `BarLowered` puts it 0.35 m lower, blocks mode restores the built pose. Blocks mode: no camera block
  overlaps the bar (projected separating-axis test, min gap 0.18; with 3 cameras the lowest block bottom is
  at −1.12 m, the layout is width-bound so it does not reach `minBottom` −1.75). LayoutVerifier needed no
  change (it projects the bar's real corners). Panel labels fit in one line.
- **Toggle bug (fixed in this round, `ImageSubscriber.IsOn`)**: in first person (blocks hidden) clicking a
  camera toggle used to activate that block. Now: head off/on through the bar toggle keeps all 3 panel
  GameObjects inactive while `IsOn` follows; `ResetLayout()` while hidden keeps them inactive (all IsOn);
  a camera turned off while hidden stays hidden after `SetBlocksShown(true)` (bar toggle shows off), the
  others come back.
- **Robustness**: RoundTrip, ArmTargets and BaseVelocity subscribed once per app run (static flag); after
  "Back to robots" the next robot screen has a new ROSConnection and they never subscribed again. They now
  subscribe once per ROSConnection (checked with a second Play session: RTT and targets arrive).
- **Polish round**: strip text, chip offset, bar drop 0.50, Rename button outside the card (picture 14).
- **Device (Quest 3S), v2 round**: TOTAL PSS 1.32 / 1.26 GB at 30 / 60 s (no lowmemorykiller); VrApi
  62.9 fps mean (min 55) while recording 10 JPEG frames/s in first person, 70.6 (min 62) after (72 Hz);
  RTT 13–27 ms (max 79); TF 37–43 Hz; no Unity exceptions. Frames show the labelled strip, both hand
  cards outboard of the grippers below the image, and the translucent arrow during the burst. The
  "Targets" log showed 5.6–5.9 m errors for the whole run: with control on, the teleop node publishes
  `*_target_link` from the (idle) controllers, so the markers were far away (not the test TFs).
- **Device (Quest 3S), v1 round**: TOTAL PSS 1.31 / 1.26 / 1.26 GB at 30 / 60 / 90 s (flat, no lowmemorykiller);
  VrApi 61.5 fps mean (min 57) while recording 10 JPEG frames/s in first person, 69.8 (min 58) after
  recording (72 Hz display); image fps 8–12 while recording, 14 after; no Unity exceptions. Frames show
  the strip, the hand-cam cards, both target markers with lines, and the arrow tip during
  the burst (the recorder looks level, so the floor arrow is mostly below its view).

## Feedback round 2

1. **fps badge in first person**: the head-camera card and the hand cards show the same "15 fps" / "stale 1.2 s"
   badge as the camera blocks (`CameraBadge` in CameraPanel.cs, fed from `ImageSubscriber.Fps` / `LastFrameTime`, ~5 Hz).
2. **Camera toggles in first person**: the bar's camera checkboxes also show/hide the head-camera card (with its
   "Waiting" label) and each hand card. `ImageSubscriber.CameraVisibilityChanged(index, on)`; a camera that is
   off in first person is no longer decoded (ForceDecode dropped), and decoded again when turned on.
3. **No topic line on camera blocks**: only setup mode (new robots, auto-generated names) shows it; the cards
   are shorter and the layout measures them as before. Rename and the name label are unchanged.
4. **Status strip in both modes**, active only while the bar is hidden. Blocks mode: no model, so no HEAD/LIFT
   and "TF —"; it sits at the bar's bottom row (4.3 m, text as large as the bar's). First person: as before.
5. **Controller and hand visuals hidden in first person while the bar is hidden** (renderers, ray line visual,
   hand mesh controller; objects and interactors stay active, so the menu button and gesture still work).
   Logged as `[TeleopHud] controller visuals hidden/shown`.

**Status: tested (harness) + built.** A new Editor harness against the live sim (ExperimentsVerify3, 40 checks) passed:
fps badges on the head card and hand cards ("15 fps", "stale 2.7 s" when the head camera is throttled to 0.2 fps);
camera toggles in first person (head card hidden and no frames decoded while off, hand card count follows,
Reset layout turns all back on, camera blocks stay hidden throughout); no topic line outside setup mode (and present
in setup mode, AddRobotTest), with matching layout metrics and no overlaps; status strip in blocks mode (no model,
"TF —") active exactly while the bar is hidden, below the blocks and at the bar's place, also after a bar rebuild;
controller/hand renderers off in first person while the bar is hidden, back on with the bar or in blocks mode,
with objects and interactors left active. The earlier harnesses still pass (ExperimentsVerify 60, ExperimentsVerify2 84,
FirstPersonVerify 50, LayoutVerifier 107, AddRobotTest 40, SelectionShot 10). The APK was rebuilt and installed on the Quest.
The hidden controllers can't be shown in the Editor (no tracked controllers), so they still need a check on the headset.

Pictures: `15_fp_badges.png` (first person, badges on the head card's top right and on both hand cards),
`16_strip_blocks_mode.png` (blocks mode, bar hidden, strip under the blocks), `17_blocks_no_topic.png`
(blocks without topic lines, layout mode with Rename shown).
