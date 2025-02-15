# joy_to_cmdvel
## Discription
Subscribe to the `/joy` topic and publish `/cmd_vel`
## How to try
```
$ sudo apt install ros-humble-joy
$ cd ros2_ws/src
$ git clone https://github.com/iHaruruki/joy_to_cmdvel.git # clone this package
$ cd ros2_ws
$ colcon build
$ source install/setup.bash
$ ros2 run joy joy_node
$ ros2 run joy_to_cmdvel_node
```
# 日本語ver
## 概要
ROS2 humble用 DualShock4コントローラー
`/joy`トピックを購読して、`/cmd_vel`を出版するノード
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
3. 操作方法
* 左アナログスティック(LS)
    * 上：[twist.linear.x ++] 前進
    * 下：[twist.linear.x --] 後退
    * 右：[twist.angular.z --] 右旋回（時計回り）
    * 左：[twist.angular.z ++] 左旋回（反時計回り）
* 右アナログスティック(RS)
    * 右：[twist.linear.y --] 右移動
    * 左：[twist.linear.y ++] 左移動
* R1 Button
    * OFF(msg->buttons[5] == 0)：通常モード
    * ON (msg->buttons[5] == 1)：高速モード<span style="color:red;">（速度を通常モードの1.5倍にする）<span>
### 参考資料
[joy_node]に関する説明
* [joy](https://docs.ros.org/en/humble/p/joy/index.html)
* [ROS2でPS4のジョイスティック（DualShock4）を使ってみました](https://kanpapa.com/today/2022/09/ros2-joy-ps4-dualshock.html)

## 更新履歴
* 2025/01/15: [README.md を作成]
