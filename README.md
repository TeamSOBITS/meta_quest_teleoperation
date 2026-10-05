<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# META QUEST TELEOPERATION

<!-- 概要 -->
## Overview

<!-- ![META QUEST TELEOPERATION](meta_quest_teleoperation/docs/img/meta_quest_teleoperation.png) -->

This package runs a Unity application (SOBITS Quest Teleoperation) on Meta Quest to communicate with ROS.
On the ROS side it uses [TeamSOBITS/ros_tcp_endpoint](https://github.com/TeamSOBITS/ros_tcp_endpoint), launched by `sobits_teleop`. The app publishes `/<ns>/joy` (controller buttons and sticks) and TF (`hmd_odom`, `left_controller_odom`, `right_controller_odom` under `base_footprint`), and subscribes to the camera topics and `/tf`.

**Robot selection screen**
- Edit the ROS IP with the Quest keyboard (Edit button)
- Robot cards (a green dot after the name when the robot is online, a "Last used" tag on the previous robot); point and pull the trigger to select
- "Add robot": type a name, then discover and pick its cameras (Find cameras)
- Robots added on the headset have a "Remove" button on their card (press again within a few seconds to confirm)
- Display settings: text size and high contrast

<p align="center"><img src="media/readme/selection.png" width="640"><br><sub>Robot selection screen (SOBIT HOME online, SOBIT LIGHT offline, Add robot).</sub></p>
<p align="center"><img src="media/readme/robot_blocks.png" width="640"><br><sub>Robot screen: camera blocks on an arc with the HUD bar.</sub></p>

**Robot screen**
- Camera images are shown as blocks on an arc. In layout mode (Control robot off) drag, resize and rename (Rename) them with the controllers or your hands
- HUD bar: Control robot (starts publishing `/<ns>/joy` and TF after a 2 s countdown), Lazy follow, Passthrough, Compressed, camera toggles, Robot model, Camera layout (Blocks / First person), Reset layout, Recenter, ← Robots (back to selection)
- The menu button or a palm-facing pinch gesture shows/hides the HUD bar; a status strip is shown while the bar is hidden
- First-person layout: head camera at its true field of view, hand-camera cards, arm target markers, base velocity arrow, head-lag outline

<p align="center"><img src="media/readme/first_person.png" width="640"><br><sub>First-person layout: head camera at its true field of view.</sub></p>
<p align="center"><img src="media/readme/overlay_targets.png" width="640"><br><sub>Overlays: arm target markers (blue = left, orange = right).</sub></p>
<p align="center"><img src="media/readme/overlay_basevel.png" width="640"><br><sub>Overlays: hand-camera cards and the base velocity arrow.</sub></p>
<p align="center"><img src="media/readme/bar.png" width="640"><br><sub>HUD bar close-up.</sub></p>

#### VLA status
The HUD shows the status of the sobits_vla_tools stage that is running, from `sobits_interfaces/VlaStatus` on `/<ns>/vla_rosbag_collection/status` (data collection, `sobits_vla_rosbag_collection`) and `/<ns>/sobits_vla_deploy/status` (policy deploy, `sobits_vla_deploy`):
- Collection: the recorder state (IDLE / REC / PAUSED / ERROR), the elapsed time, the task (`task: pick cup`), and short save / discard / delete / task notices.
- Deploy: PLAY with the episode time (the clock starts at the first grip), RESETTING while the world resets, the task and the policy's short name (`pick cup · smolvla_fft`), a GRIP chip while the deadman is enabled and the policy plays (`GRIP driving` green, `GRIP released` amber), and the episode's steps and inference rate (`240 steps · 8.1 Hz`). The episode outcome is not shown.

It appears as a VLA group in the status strip while the menu is hidden, a pill in the menu header while it is open (its dot takes the GRIP colour during deploy), and the notices just above the strip (or above the menu); only while a stage node is running (the badge hides when no message arrived for 3 s). If both nodes publish, an idle heartbeat of one never replaces the other's live status. The topics are the profile's `vlaStatusSuffixes`. The headset only displays: Quest A/B/X drive the recorder or the deploy node through sobits_vla_tools' `GamepadClient`.
Test it without ROS: `tools/verify.sh --suite VlaStatusVerify`, or `tools/device.sh launch --recstatus [deploy]` (fake collection / deploy feed on the headset; `tools/device.sh logs` prints the `VLA:` lines).

<p align="right">(<a href="#readme-top">Back to top</a>)</p>


<!-- 環境構築 -->
## Setup

This section explains how to set up this repository.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>


### Requirements

Prepare the following environments before installation.

ROS PC (machine running ROS)
| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.10+ |

Unity build environment (for building the Meta Quest Unity app)
| System  | Version |
| --- | --- |
| Windows 11 / macOS / Linux | Any (as long as Unity runs)

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

### Install Unity Hub

Unity Hub is available on all major OSes. Below is how to install it on Linux (Ubuntu). Refer to the official guide: [Install the Unity Hub on Linux](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux).

1. Add the public key:
    ```sh
    $ wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
    ```

2. Add the Unity Hub repository to `/etc/apt/sources.list.d`:
    ```sh
    $ sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
    ```

3. Install Unity Hub:
    ```sh
    $ sudo apt update
    $ sudo apt-get install unityhub
    ```

<p align="right">(<a href="#readme-top">Back to top</a>)</p>


### Open the Unity Project

1. Clone this repository to any directory:
    ```sh
    $ git clone https://github.com/TeamSOBITS/meta_quest_teleoperation.git
    ```

2. In Unity Hub, go to `Projects -> ADD -> Add project from disk` and select this repository's `UnityProject`. Unity Hub will install the appropriate Unity Editor version. Ensure Android Build Support is selected.

   The Unity version is `6000.0.69f1` (`UnityProject/ProjectSettings/ProjectVersion.txt`).

3. Once Unity opens, go to `Edit -> Project Settings -> XR Plugin Management` and check **OpenXR only** on both the PC and Android tabs (Oculus is not used).

4. Under `XR Plugin Management -> OpenXR` (Android tab), enable the following features (already set in this project):
   - Meta Quest Support
   - Oculus Touch Controller Profile / Meta Quest Touch Plus Controller Profile
   - Hand Interaction Profile
   - Hand Tracking Subsystem

   Packages used: OpenXR Plugin 1.14, OpenXR: Meta 2.1, AR Foundation (passthrough), XR Hands 1.5, XR Interaction Toolkit 3.1.1, ROS TCP Connector.

5. For the overlay keyboard (IP and name entry), `Assets/Plugins/Android/AndroidManifest.xml` declares the `oculus.software.overlay_keyboard` uses-feature. No change is needed.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

### Setup ROS PC
1. Navigate to your ROS `src` folder:
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone ROS TCP Endpoint:
    ```sh
    $ git clone https://github.com/TeamSOBITS/ros_tcp_endpoint.git
    ```

3. Build the packages:
   ```bash
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

## Build
After setting up `meta_quest_teleoperation`, verify Unity-ROS Connection and build the app to your Meta Quest device.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

### Unity-ROS Connection
Start the TCP Endpoint on the ROS side through `sobits_teleop`:
```sh
$ ros2 launch sobits_teleop sobits_teleop.launch.py device:=quest use_sim_time:=true use_moveit:=true use_servo:=true
```
For SOBIT LIGHT, add `robot_name:=sobit_light`. `use_sim_time:=true` is for the simulation (Gazebo).
The ROS IP is set inside the app (robot selection screen), not in the Unity scene, so nothing needs to be configured in Unity.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

### Build the Unity App
Build with the Unity menu `Robots -> Build APK`, or with `tools/verify.sh --build` (headless, on a scratch copy). The APK is written to `UnityProject/Builds/SOBITS-Quest-Teleoperation-<version>.apk`.
Connect Meta Quest to your PC via USB, allow the prompt on the headset, then install:
```sh
$ adb install -r UnityProject/Builds/SOBITS-Quest-Teleoperation-<version>.apk
```
(`File -> Build Profiles -> Android -> Build and Run` also deploys directly to the headset.)

The C# classes of the `sobits_interfaces` messages the app uses live in `UnityProject/Assets/RosMessages/` (generated). After the `.msg` changes, regenerate them with `tools/gen_msgs.sh [SOBITS_INTERFACES_DIR]` (the Unity Editor must not have the project open; or the menu `Robots -> Generate sobits_interfaces messages`).

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

### Meta Quest-ROS Connection
On Meta Quest, open "SOBITS Quest Teleoperation" under "Unknown Sources" while the ROS TCP Endpoint is running.

- USB: run `adb reverse tcp:10000 tcp:10000` on the PC and set the app's ROS IP to `127.0.0.1`.
- Wi-Fi: enter the ROS PC's IP as the app's ROS IP (port 10000).

Change the IP with the Edit button on the robot selection screen or on the robot screen's HUD bar.
The app then sends the controller states (`/<ns>/joy`, `sensor_msgs/Joy`) and pose information (`/tf`, `tf2_msgs/TFMessage`) to ROS and receives the camera images.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

## Adding a Robot Model
Steps to add a robot model.

<p align="center"><img src="media/readme/light_model.png" width="640"><br><sub>SOBIT LIGHT model as shown in the app (side view).</sub></p>

1. Put the following in `tools/models/<robot>/`:
   - `<robot>.urdf`: URDF generated from xacro
   - `src/`: only the meshes the URDF references (git-ignored). Own package under `src/meshes/<rel>`, other packages under `src/ext/<pkg>/<rel>`
   - `budget.json`: per-mesh triangle targets
2. Decimate the meshes (a venv with `numpy trimesh fast-simplification pycollada` is required):
   ```sh
   $ python3 tools/decimate_meshes.py --robot <robot>
   ```
   Output goes to `tools/models/<robot>/meshes_lod/`.
3. In Unity, run the menu `Robots -> Build <robot> model` to generate the model prefab (currently SOBIT HOME / SOBIT LIGHT).
4. Fill in the `RobotProfile` asset (frames: camera, pan, tilt; cameras; arms; `odomSuffix`; `vlaStatusSuffixes`: status topics of the VLA stage nodes relative to the namespace, `vla_rosbag_collection/status` and `sobits_vla_deploy/status`, empty = no VLA badge).
5. Run `Robots -> Validate profiles`.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

## Testing
Tests run against the Gazebo simulations in the ROS container.

- `tools/sim.sh home|light|status`: start the simulation and `sobits_teleop` (SOBIT HOME / SOBIT LIGHT) and check its status
- `tools/verify.sh --sync --all`: sync to a scratch copy and run all Editor verification suites headlessly (the Unity Editor must not have the project open)
- `tools/device.sh launch [--robot SOBIT_HOME|SOBIT_LIGHT] [--viewmode blocks|model|firstperson] ...`: launch the app on the headset (`logs`, `stop`, etc. are also available)

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

## Milestones

- [ ] Add pseudo inverse kinematics
- [ ] Add inverse kinematics on Meta Quest

Check the [Issues page][issues-url] for current bugs and feature requests.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

## References
- [Meta Quest for Teleop Setup Guide](https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/)

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/meta_quest_teleoperation.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/meta_quest_teleoperation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/meta_quest_teleoperation.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/meta_quest_teleoperation/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/meta_quest_teleoperation.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/meta_quest_teleoperation/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/meta_quest_teleoperation.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/meta_quest_teleoperation/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/meta_quest_teleoperation.svg?style=for-the-badge
[license-url]: LICENSE



<!-- LICENSE -->
## License

BSD-3-Clause, see [LICENSE](LICENSE). This project is a fork of [PickNikRobotics/meta_quest_teleoperation](https://github.com/PickNikRobotics/meta_quest_teleoperation).
