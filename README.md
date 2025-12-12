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
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
        <ul>
            <li><a href="#環境条件">環境条件</a></li>
            <li><a href="#unity-hubをインストール">Unity Hubをインストール</a></li>
            <li><a href="#unityプロジェクトを開く">Unityプロジェクトを開く</a></li>
            <li><a href="#ros-pcのセットアップ">ROS PCのセットアップ</a></li>
        </ul>
    </li>
    <li>
    　<a href="#ビルド方法">ビルド方法</a>
      <ul>
        <li><a href="#Unityアプリをビルド">Unityアプリをビルド</a></li>
        <li><a href="#Unityアプリを実行">Unityアプリを実行</a></li>
        <li><a href="#meta-questとros通信">Meta QuestとROS通信</a></li>
      </ul>
    </li>
    <li><a href="#参考文献">参考文献</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>



<!-- 概要 -->
## 概要

<!-- ![META QUEST TELEOPERATION](meta_quest_teleoperation/docs/img/meta_quest_teleoperation.png) -->

ROSと通信するためのUnityアプリをMeta Quest上で動作させるためのパッケージ．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

ROS PC（ROSを実行するマシン）
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Noble Numbat) |
| ROS    | Humble Hawksbill|
| Python | 3.10~ |

Unityアプリビルド用（Meta Quest用Unityアプリをビルドする環境）
| System  | Version |
| --- | --- |
| Windows 11 / macOS / Linux | 任意（Unity が動作する環境であれば可）

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Unity Hubをインストール

Unity HubはどのOSでも使用できますが，ここではLinux(Ubuntu)にUnity Hubをインストールする方法を紹介します．
[Unity HubをLinuxにインストールする](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux)を参考にUbuntuにUnity Hubをインストールします．

1. 公開鍵を追加します．
    ```sh
    $ wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
    ```

2. Unity Hub のリポジトリ情報を`/etc/apt/sources.list.d`に追加します．
    ```sh
    $ sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
    ```

3. Unity Hubをインストールします．
    ```sh
    $ sudo apt update
    $ sudo apt-get install unityhub
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Unityプロジェクトを開く

1. 任意のディレクトリに本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/meta_quest_teleoperation
    ```

2. Unity Hubの`Projects` -> `ADD` -> `Add project from disk`で本レポジトリ（UnityProject）を選択します．この時点で本レポジトリに合わせたバージョンのUnity Editorがインストールされます．Android Build Supportにチェックを入れ，インストールします．

3. Unityが起動できたら，`Edit` -> `Project Settings`で`XR Plugin Management`へ移動し，すべての項目でOpenXRのチェックを外し，Oculusにチェックを入れます．

4. `XR Plugin Management` -> O`culus`のTarget Devicesで使用するMeta Questのバージョンを指定します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### ROS PCのセットアップ
1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. ROS TCP Endpointをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/ros_tcp_endpoint
    ```

3. パッケージをコンパイルします．
   ```bash
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## ビルド方法
meta_quest_teleoperationのセットアップが完了したら，ROSと通信できることを確認し，Meta Questデバイスへアプリをビルドします．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### UnityとROSの通信
ROS側でTCP Endpointを起動します．ros_ipにPCのIPを指定してください．
```sh
$ ros2 launch ros_tcp_endpoint endpoint.launch.py ros_ip:=192.168.XXX.XXX
```
次に，Unity側でProjectウィンドウから`Assets -> Scenes`へ移動し，TeleopSceneを選択します．HierarchyウィンドウでROSTCPConnectorを選択し，InspectorウィンドウのROS ConnectionスクリプトにあるROS IP AddressをROSのIPと同じ値を設定します．

Unityのシーン内に表示されている通信の矢印が青くなったら成功です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Unityアプリをビルド
通信が確認できたらUnityアプリをMeta Questへビルドします．
USBケーブルでPCとMeta Questを接続し，Meta Quest内に表示される共有を許可します．
Unityの`File -> Build Profiles -> Android -> Run Device`にMeta Questが表示されるので選択します．
`Build and Run`をクリックし，アプリをビルドします．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Meta QuestとROS通信
Meta Questのアプリ一覧で「提供元不明のアプリ」にUnityアプリが表示されるのでアプリをクリックし，同時にROS TCP Endpointを起動します．
これで，Meta Questのボタン状態（sensor_msgs/Joy型），位置姿勢情報（tf2_msgs/TFMessage型）をROS側に送信することが可能です．
アプリ実行後のIP変更はMeta Questコントローラのメニューボタンを押し，IPを入力します．


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
- [Meta Quest for Teleop Setup Guide](https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン

- [ ] 疑似逆運動学の追加
- [ ] Meta Questでの逆運動学の追加

現時点のバッグや新規機能の依頼を確[text](../sobit_edu)認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

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

