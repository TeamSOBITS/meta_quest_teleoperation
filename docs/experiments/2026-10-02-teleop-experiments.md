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
| 4 | `handcams` | **Hand cameras in first person**: PiP cards floating above each gripper (`hand_*_end_effector_link`), facing the head; option "corners" = head-locked bottom corners. | Grasping needs the wrist view; blocks are hidden in first person. | at hands | Screenshot with the sim hand cams live; cards follow the wrists when an arm moves. |
| 5 | `targets` | **Arm target markers**: small spheres at `left_target_link` / `right_target_link` (from `/tf`, parent base frame) + a line to the model's end effector. | Shows tracking error and lag between where I point and where the arm is. | on | Container publishes static target TFs; screenshot shows markers + lines; distance logged. |
| 6 | `basevel` | **Base velocity arrow** on the floor at the model base from `/sobit_home/odom` twist. | Driving blind in first person; shows actual motion incl. lag. | on | Container sends `cmd_vel` for 2 s; arrow appears/scales; screenshot. |
| 7 | `deadman` | **Grip to control**: with Control on, TF/Joy are published only while a grip button is held; strip shows HOLD GRIP. | Safety: no accidental motion from a stray head turn. | off | Harness: no publish without grip (publisher counter), publish with simulated grip. |
| 8 | `headlock` | **Image follows my head** (instead of the robot's head frame). | Hides head lag for comfort; the model still shows the real head. | off | Screenshot: quad centred while the robot head is panned 0.5 rad. |
| 9 | `menurecenter` | **Long-press menu (1.5 s) = Recenter** in first person. | Recenter without opening the bar. | on | Harness simulates the long press, asserts a recenter log. |
| 10 | *(design)* | Key light on the model; in first person the bar opens lower and narrower so it never covers the image centre; Experiments panel styled like the bar. | Model was near-black; bar covered the image. | — | Screenshots before/after. |

## Status
Filled in as work lands (agent updates this table): **planned → implemented → tested (harness) → tested (device)**.

| # | Status | Picture |
|---|--------|---------|
| 1 | tested (harness), tested (device) | `shots/01_latest_frame.png` |
| 2 | tested (harness) | `shots/02_rtt_strip.png` |
| 3 | tested (harness), tested (device) | `shots/03_status_strip.png`, `shots/11_device_strip.png` |
| 4 | tested (harness), tested (device) | `shots/04_handcams_hands.png`, `shots/04_handcams_corners.png`, `shots/11_device_strip.png` |
| 5 | tested (harness), tested (device) | `shots/05_targets.png`, `shots/12_device_targets.png` |
| 6 | tested (harness), tested (device) | `shots/06_basevel.png` |
| 7 | tested (harness), tested (device) | `shots/07_deadman.png`, `shots/11_device_strip.png` |
| 8 | tested (harness), tested (device) | `shots/08_headlock.png` |
| 9 | tested (harness) | — (log-only check) |
| 10 | tested (harness) | `shots/10_bar_compact.png`, `shots/10_experiments_panel.png`, `shots/10_key_light.png` |

Harness = Editor Play mode in a scratch copy of the project against the live SOBIT HOME sim
(`ExperimentsVerify` batch 1: 42/42, `ExperimentsVerify2` batches 2–3: 73/73; regressions
LayoutVerifier 107/0, AddRobotTest 38/0, SelectionShot 10/0, FirstPersonVerify 50/50). Device = Quest 3S,
IL2CPP build, 45 s on-device recording (`--ei record 45 --ei fps 10 --ei switchat 5`) with every
experiment on, while the container moved the head, published both arm targets for 10 s, sent a
cmd_vel burst and moved the left arm. Experiment toggles can be set at launch for such tests:
`--es exp "deadman=1,headlock=1"` (`key=0|1`, comma separated; saved like a click in the panel).

## Findings
- **1 Latest frame**: head camera at 25 Hz with maxFps 15 shows 15 fps (badge 14–16), the rest is
  dropped (+51 in 5 s); at maxFps 2 the shown frame was the newest received message 12/12 times,
  interval 0.500 s, +142 drops in 6 s. Decode 1.2–1.4 ms per frame in the Editor, 2.8–4.8 ms on the Quest.
- **2 RTT**: 7–11 ms smoothed (max over 1 s ≈ 41 ms) against the local endpoint, using an injected
  `hmd_odom` TF (the Editor has no XR head, so the publisher itself sends nothing); stops 2.0 s after
  control off. Not measured on the device: with "Grip to control" on and nobody holding a grip
  nothing is published, so the strip correctly shows "RTT —".
- **3 Strip**: pan/tilt read back exactly (0.500 / +0.300 rad, tilt marker up), lift 0.400 m. The bold
  CONTROL ON / HOLD GRIP label overflowed its pill into the fps text; it now shrinks to fit.
- **4 Hand cams**: 2 cards, frames within 5 s at 15 fps each with the blocks hidden; the left card
  moved 0.56 m with the arm; corners mode parents them under the head; off clears their forced
  decoding (0 hand frames in 2 s). Corner cards overlapped the status strip at x = ±0.45 m: moved to ±0.52 m.
- **5 Targets**: marker at (∓0.2, 0.9, 0.4) in model-root space to < 1 mm for ROS (0.4, ±0.2, 0.9);
  error = |marker − end effector| (0.24 m at the home pose); hidden 0.7–0.85 s after the publisher stops.
- **6 Base velocity**: Linear.x 0.15, Angular 0.50; arrow along the base's forward (dot 1.000); hidden
  0.4 s after odom reports the base still. The arrow now starts 0.35 m ahead of the base centre (it was
  hidden under the robot's own base seen from the head) and the turn arc radius is 0.55 m. Note: the sim
  base keeps its last cmd_vel, so a test must send a zero Twist.
- **7 Grip to control**: without a grip nothing is published for 2 s, strip and bar chip say HOLD GRIP;
  a (simulated) grip flips DeadmanHeld within 0.05 s and the chip to CONTROL ON. Actual publishing
  while held needs a tracked XR head, so it is not exercised in the Editor.
- **8 Image follows my head**: the image hangs at (0, 0, 1.5) under the head and stays centred while the
  model's head pans 28.6°; off puts it back on `head_camera_color_frame`.
- **9 Long-press menu**: a 2 s hold logs one recenter and leaves the bar alone; a 0.3 s press toggles the
  bar once on release; with the experiment off the bar toggles on the press.
- **10 Bar / panel / light**: in first person the bar is 0.7× and 0.35 m lower, restored in blocks mode;
  panel labels fit in one line. The status strip's right end touches the Experiments panel's top-left
  corner when the compact bar is open (cosmetic).
- **Robustness**: RoundTrip, ArmTargets and BaseVelocity subscribed once per app run (static flag); after
  "Back to robots" the next robot screen has a new ROSConnection and they never subscribed again. They now
  subscribe once per ROSConnection (checked with a second Play session: RTT and targets arrive).
- **Device (Quest 3S)**: TOTAL PSS 1.31 / 1.26 / 1.26 GB at 30 / 60 / 90 s (flat, no lowmemorykiller);
  VrApi 61.5 fps mean (min 57) while recording 10 JPEG frames/s in first person, 69.8 (min 58) after
  recording (72 Hz display); image fps 8–12 while recording, 14 after; no Unity exceptions. Frames show
  the strip with HOLD GRIP, the hand-cam cards, both target markers with lines, and the arrow tip during
  the burst (the recorder looks level, so the floor arrow is mostly below its view).
