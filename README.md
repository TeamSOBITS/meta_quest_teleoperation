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
        <li><a href="#unityとros通信">UnityとROS通信</a></li>
        <li><a href="#unityアプリをビルド">Unityアプリをビルド</a></li>
        <li><a href="#meta-questとros通信">Meta QuestとROS通信</a></li>
      </ul>
    </li>
    <li><a href="#ロボットモデルの追加">ロボットモデルの追加</a></li>
    <li><a href="#テスト">テスト</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>



<!-- 概要 -->
## 概要

<!-- ![META QUEST TELEOPERATION](meta_quest_teleoperation/docs/img/meta_quest_teleoperation.png) -->

ROSと通信するためのUnityアプリ（SOBITS Quest Teleoperation）をMeta Quest上で動作させるためのパッケージ．
ROS側は[TeamSOBITS/ros_tcp_endpoint](https://github.com/TeamSOBITS/ros_tcp_endpoint)を`sobits_teleop`から起動して使用します．アプリは`/<ns>/joy`（コントローラのボタン・スティック）とTF（`base_footprint`配下の`hmd_odom`，`left_controller_odom`，`right_controller_odom`）をROSへ送信し，カメラ画像のトピックと`/tf`を購読します．

**ロボット選択画面**
- ROS IPをQuestのキーボードで編集（Editボタン）
- ロボットのカード（オンラインならカード名の横に緑のドット，前回使用したロボットには「Last used」タグ）．ポインタを向けてトリガーで選択
- 「Add robot」: 名前を入力して追加し，カメラを自動検出（Find cameras）して選択
- 追加したロボットのカードには「Remove」ボタン（数秒以内にもう一度押して確定）
- Display settings: 文字サイズとハイコントラスト

**ロボット画面**
- カメラ画像はアーチ状に並んだブロックとして表示．レイアウトモード（Control robotオフ）でコントローラまたは手でドラッグ・サイズ変更・名前変更（Rename）が可能
- HUDバー: Control robot（オンにすると2秒のカウントダウン後に`/<ns>/joy`とTFの送信を開始），Lazy follow，Passthrough，Compressed，各カメラのON/OFF，Robot model，Camera layout（Blocks / First person），Reset layout，Recenter，← Robots（ロボット選択へ戻る）
- メニューボタンまたは手のひらを向けてピンチするジェスチャでHUDバーを表示/非表示．バー非表示中はステータスストリップを表示
- First personレイアウト: 頭部カメラを実際の視野角で表示，ハンドカメラのカード，アームの目標マーカー，ベース速度の矢印，頭部の遅れを示すアウトライン

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
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
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
    $ git clone https://github.com/TeamSOBITS/meta_quest_teleoperation.git
    ```

2. Unity Hubの`Projects -> ADD -> Add project from disk`で本レポジトリ（UnityProject）を選択します．この時点で本レポジトリに合わせたバージョンのUnity Editorがインストールされます．Android Build Supportにチェックを入れ，インストールします．

   Unityのバージョンは`6000.0.69f1`（`UnityProject/ProjectSettings/ProjectVersion.txt`）です．

3. Unityが起動できたら，`Edit -> Project Settings -> XR Plugin Management`へ移動し，PC / Androidタブともに**OpenXRのみ**にチェックを入れます（Oculusは使用しません）．

4. `XR Plugin Management -> OpenXR`のFeature Group（Androidタブ）で以下を有効にします（本プロジェクトは設定済み）．
   - Meta Quest Support
   - Oculus Touch Controller Profile / Meta Quest Touch Plus Controller Profile
   - Hand Interaction Profile
   - Hand Tracking Subsystem

   使用パッケージ: OpenXR Plugin 1.14，OpenXR: Meta 2.1，AR Foundation（パススルー用），XR Hands 1.5，XR Interaction Toolkit 3.1.1，ROS TCP Connector．

5. オーバーレイキーボード（IP・名前入力用）のため，`Assets/Plugins/Android/AndroidManifest.xml`に`oculus.software.overlay_keyboard`の`uses-feature`を記述しています．変更は不要です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### ROS PCのセットアップ
1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. ROS TCP Endpointをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/ros_tcp_endpoint.git
    ```

3. パッケージをコンパイルします．
   ```bash
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## ビルド方法
`meta_quest_teleoperation`のセットアップが完了したら，ROSとUnityの通信を確認し，Meta Questデバイスへアプリをビルドします．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### UnityとROS通信
ROS側で`sobits_teleop`からTCP Endpointを起動します．
```sh
$ ros2 launch sobits_teleop sobits_teleop.launch.py device:=quest use_sim_time:=true use_moveit:=true use_servo:=true
```
SOBIT LIGHTの場合は`robot_name:=sobit_light`を追加します．`use_sim_time:=true`はシミュレーション（Gazebo）用です．
接続先のIPはUnityのシーンではなくアプリ内（ロボット選択画面のROS IP）で設定するため，Unity側での設定は不要です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Unityアプリをビルド
Unityのメニュー`Robots -> Build APK`，またはコマンド`tools/verify.sh --build`でビルドします（後者は専用のコピーでヘッドレス実行）．APKは`UnityProject/Builds/SOBITS-Quest-Teleoperation-<version>.apk`に出力されます．
Meta QuestをUSBで接続し，ヘッドセット内の接続許可を承認してからインストールします．
```sh
$ adb install -r UnityProject/Builds/SOBITS-Quest-Teleoperation-<version>.apk
```
（Unityの`File -> Build Profiles -> Android -> Build and Run`でも直接実機へ転送できます．）

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Meta QuestとROS通信
Meta Questのアプリ一覧で「提供元不明のアプリ」から「SOBITS Quest Teleoperation」を起動し，ROS側でTCP Endpointを起動しておきます．

- USB接続: PCで`adb reverse tcp:10000 tcp:10000`を実行し，アプリのROS IPを`127.0.0.1`にします．
- Wi-Fi接続: アプリのROS IPにROS PCのIPを入力します（ポートは10000）．

IPはロボット選択画面のEditボタン，またはロボット画面のHUDバーのEditボタンから変更できます．
これで，Meta Questのボタン状態（`/<ns>/joy`，sensor_msgs/Joy型），位置姿勢情報（`/tf`，tf2_msgs/TFMessage型）をROS側に送信し，カメラ画像を受信できます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## ロボットモデルの追加
ロボットのモデルを追加する手順です．

1. `tools/models/<robot>/`に以下を置きます．
   - `<robot>.urdf`: xacroから生成したURDF
   - `src/`: URDFが参照するメッシュのみ（git管理外）．自パッケージは`src/meshes/<rel>`，外部パッケージは`src/ext/<pkg>/<rel>`
   - `budget.json`: メッシュごとの三角形数の目標
2. メッシュを軽量化します（venvに`numpy trimesh fast-simplification pycollada`が必要）．
   ```sh
   $ python3 tools/decimate_meshes.py --robot <robot>
   ```
   出力は`tools/models/<robot>/meshes_lod/`です．
3. Unityのメニュー`Robots -> Build <robot> model`でモデルのプレハブを生成します（現在はSOBIT HOME / SOBIT LIGHT）．
4. `RobotProfile`アセットを埋めます（フレーム: カメラ・パン・チルト，カメラ，アームなど）．
5. `Robots -> Validate profiles`でプロファイルを検証します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## テスト
テストはROSコンテナ内のGazeboシミュレーションに対して実行します．

- `tools/sim.sh home|light|status`: シミュレーションと`sobits_teleop`の起動（SOBIT HOME / SOBIT LIGHT）と状態確認
- `tools/verify.sh --sync --all`: スクラッチコピーに同期してEditor検証スイートをヘッドレスで全実行（Unity Editorでプロジェクトを開いていない状態で実行）
- `tools/device.sh launch [--robot SOBIT_HOME|SOBIT_LIGHT] [--viewmode blocks|model|firstperson] ...`: 実機でアプリを起動（`logs`，`stop`なども利用可）

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン

- [ ] 疑似逆運動学の追加
- [ ] Meta Questでの逆運動学の追加

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
- [Meta Quest for Teleop Setup Guide](https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/)

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

