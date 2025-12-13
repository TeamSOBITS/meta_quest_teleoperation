<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# META QUEST TELEOPERATION

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#overview">Overview</a></li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#requirements">Requirements</a></li>
        <li><a href="#install-unity-hub">Install Unity Hub</a></li>
        <li><a href="#open-the-unity-project">Open the Unity Project</a></li>
        <li><a href="#setup-ros-pc">Setup ROS PC</a></li>
      </ul>
    </li>
    <li>
      <a href="#build">Build</a>
      <ul>
        <li><a href="#unity-ros-connection">Unity-ROS Connection</a></li>
        <li><a href="#build-the-unity-app">Build the Unity App</a></li>
        <li><a href="#meta-quest-ros-connection">Meta Quest-ROS Connection</a></li>
      </ul>
    </li>
    <li><a href="#milestones">Milestones</a></li>
    <li><a href="#references">References</a></li>
  </ol>
  </details>



<!-- 概要 -->
## Overview

<!-- ![META QUEST TELEOPERATION](meta_quest_teleoperation/docs/img/meta_quest_teleoperation.png) -->

This package runs a Unity application on Meta Quest to communicate with ROS.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## Setup

This section explains how to set up this repository.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Requirements

Prepare the following environments before installation.

ROS PC (machine running ROS)
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10+ |

Unity build environment (for building the Meta Quest Unity app)
| System  | Version |
| --- | --- |
| Windows 11 / macOS / Linux | Any (as long as Unity runs)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Open the Unity Project

1. Clone this repository to any directory:
    ```sh
    $ git clone https://github.com/TeamSOBITS/meta_quest_teleoperation.git
    ```

2. In Unity Hub, go to `Projects -> ADD -> Add project from disk` and select this repository's `UnityProject`. Unity Hub will install the appropriate Unity Editor version. Ensure Android Build Support is selected.

3. Once Unity opens, go to `Edit -> Project Settings -> XR Plugin Management`. Uncheck OpenXR for all platforms and check Oculus.

4. Under `XR Plugin Management -> Oculus`, set Target Devices to match your Meta Quest version.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## Build
After setting up `meta_quest_teleoperation`, verify Unity-ROS Connection and build the app to your Meta Quest device.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Unity-ROS Connection
Start the ROS TCP Endpoint on the ROS side. Set `ros_ip` to your PC's IP:
```sh
$ ros2 launch ros_tcp_endpoint endpoint.launch.py ros_ip:=192.168.XXX.XXX
```
Then, in Unity, open `Assets -> Scenes` and select `TeleopScene`. In the Hierarchy, select `ROSTCPConnector` and in the Inspector's `ROS Connection` script, set `ROS IP Address` to the same IP. Connection is successful when the arrows shown in the scene turn blue.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Build the Unity App
After confirming connection, build the Unity app to Meta Quest.
Connect Meta Quest to your PC via USB and allow the sharing prompt on the headset.
In Unity, go to `File -> Build Profiles -> Android -> Run Device`, select your Meta Quest, and click `Build and Run`.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Meta Quest-ROS Connection
On Meta Quest, open the Unity app under "Unknown Sources" and start the ROS TCP Endpoint at the same time.
You can now send the controller button states (`sensor_msgs/Joy`) and pose information (`tf2_msgs/TFMessage`) to ROS.
To change the IP after launching, press the Meta Quest controller's menu button and enter the IP.


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## Milestones

- [ ] Add pseudo inverse kinematics
- [ ] Add inverse kinematics on Meta Quest

Check the [Issues page][issues-url] for current bugs and feature requests.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

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

