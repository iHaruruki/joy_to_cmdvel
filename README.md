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
## 更新履歴
* 2025/01/15: [README.md を作成]
