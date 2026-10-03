# SOBIT LIGHT model inputs

Source: container `jazzy_sobit_sciurus_kachaka_ws`, `~/colcon_ws/src/sobit_light` @ aea64f8. Paths below are relative to `~/colcon_ws/src/` unless noted.

## Files
- `sobit_light.urdf`: xacro output (command in its header comment). Visual meshes use `scale="0.001"` for all own-package STLs, d405.stl (mm) and `1.0`/none for kachaka STLs and DAEs (metres). Source: the generated URDF.
- `src/`: referenced meshes only (git-ignored; `tools/models/*/src/`). Own package under `src/meshes/<rel>`, external under `src/ext/<pkg>/<rel>`.
- `budget.json`: per-mesh triangle targets (own-package keys relative to `meshes/`, same format as `tools/mesh_budget.json`).

## Launch
- Sim: `ros2 launch sobit_light_bringup gz_minimal.launch.py enable_tf_prefix:=false headless:=true`
- Teleop: `ros2 launch sobits_teleop sobits_teleop.launch.py robot_name:=sobit_light device:=quest use_sim_time:=true use_moveit:=true use_servo:=true`

## Controllers (sobit_light/sobit_light_control/config/gz_controllers.yaml)
- joint_state_broadcaster (JointStateBroadcaster)
- head_position_controller (JTC): head_yaw_joint, head_pitch_joint
- arm_position_controller (JTC): arm_shoulder_roll_joint, arm_shoulder_pitch_joint, arm_elbow_pitch_joint, arm_forearm_roll_joint, arm_wrist_pitch_joint, arm_wrist_roll_joint
- hand_position_controller (JTC): hand_joint (prismatic, metres)
- wheel_controller (DiffDriveController): left base_l_drive_wheel_joint, right base_r_drive_wheel_joint; wheel_separation 0.200, wheel_radius 0.045, odom_frame odom, base_frame base_footprint
- All command interface: position (wheel: velocity via cmd_vel). Update rate 100 Hz.

## quest.yaml poses (sobits_teleop/config/sobit_light/quest.yaml, controller_poses.poses, time_from_start 3.0)
Arm joint order as above.
- servo_ready_pose (button 0): arm [0.0, -1.1, -0.6, 0.0, 0.8, 0.0]; head yaw/pitch [0.0, 0.0]
- floor_ready_pose (button 1): arm [0.0, -0.5, -0.2, 0.0, 0.7, 0.0]
- hand_open: hand_joint 0.029 (URDF upper limit); hand_close: hand_joint -0.013
- hand_grip blend open->close speed 0.002 per 50 ms tick; hand_toggle cycle on button 6 (close first, then open)
- controller_cartesian.arm: end_effector_frame_name `hand_end_effector_link`, target_frame_name `arm_target_link`, controller_frame right_controller_odom, motion_scale 0.35
- controller_tracking.head: target_frame hmd_odom, head_yaw_joint (yaw, sign 1), head_pitch_joint (pitch, sign 1)
- Same file: `arm_target_link` is NOT a URDF link (absent from sobit_light.urdf); it is a runtime TF frame from the teleop stack (referenced in sobits_teleop/scripts/tracking_test.py).

## Unity profile frames (verified against sobit_light.urdf, generated with enable_tf_prefix:=false)
- cameraFrame `head_camera_color_frame` (link exists; optical frame is `head_camera_color_optical_frame`)
- panFrame `head_yaw_link`, tiltFrame `head_pitch_link`; no lift (no lift joint; 8 revolute: 2 head + 6 arm; 3 prismatic: hand_joint, hand_sub_joint (mimic), docking_joint (kachaka docking); 2 continuous: wheels)
- arm target `arm_target_link` -> `hand_end_effector_link` (see above)
- hand camera link `hand_camera_link` (color frame `hand_camera_color_frame`, optical `hand_camera_color_optical_frame`); parent arm_wrist_roll_link (sobit_light_description/robots/sobit_light_robot.urdf.xacro:107)
- head camera parent head_pitch_link (same xacro :115)
- camera_info topic (sim): `/sobit_light/head_camera/color/camera_info` (remap in sobit_light_bringup/launch/robot.launch.py:430); hand `/sobit_light/hand_camera/color/camera_info` (:432); front/back `front_camera/camera_info`, `back_camera/camera_info` (:434, :436)
- hfov 1.21126 rad (sobit_light_description/urdf/gazebo.urdf.xacro, all active camera sensors)

## Sim camera resolutions (sobit_light_description/urdf/gazebo.urdf.xacro, active sensors in generated URDF)
- head color+depth 1280x720 (gazebo.urdf.xacro:218-219, 253-254)
- hand color+depth 640x480 (:287-288, 322-323); real hand D405/D435 profile 848x480@5 (sobit_light_bringup/launch/include/hand_cam_param.yaml: rgb_camera.color_profile "848,480,5")
- base front/back 640x480 (:110-111, :183-184); the 800x600 wideangle variants are commented out in the generated URDF
- Real head camera depth profile 640x480@15 (sobit_light_bringup/launch/include/head_cam_param.yaml:26)
