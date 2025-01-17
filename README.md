# joy_to_cmdvel
## 概要
Subscribe to the `/joy` topic and publish `/cmd_vel`
## インストール
joystick関連のパッケージをインストール
```
sudo apt install ros-humble-joy
```
## 使い方
1. `/joy`をPublishするノードを起動
```
ros2 run joy joy_node
```
2. `/joy`をSubscribeして`/cmd_del`をPublishするノードを起動
```
ros2 run joy_to_cmdvel joy_to_cmdvel_node
```
### 参考資料
[joy_node]に関する説明
* [joy](https://docs.ros.org/en/humble/p/joy/index.html)
* [ROS2でPS4のジョイスティック（DualShock4）を使ってみました](https://kanpapa.com/today/2022/09/ros2-joy-ps4-dualshock.html)

## 更新履歴
* 2025/01/15: [README.md を作成]
