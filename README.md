# joy_to_cmdvel
## Description
Subscribe to the `/joy` topic and publish `/cmd_vel`

## Setup
```shell
sudo apt install ros-humble-joy
```
```shell
cd ~/ros2_ws/src
git clone https://github.com/iHaruruki/joy_to_cmdvel.git
cd ~/ros2_ws
colcon build --packages-select joy_to_cmdvel
source install/setup.bash
```
## How to use
1. Start the node that publishes to `/joy`
```shell
ros2 run joy joy_node
```
2. Start the node that subscribes to `/joy` and publishes to `/cmd_vel`
```shell
ros2 run joy_to_cmdvel joy_to_cmdvel_node
```
3. Controls
* Left analog stick (LS)
    * Up: `twist.linear.x ++` — forward
    * Down: `twist.linear.x --` — backward
    * Right: `twist.linear.y --` — move right
    * Left: `twist.linear.y ++` — move left
* R2 button
    * Rotate right: `twist.angular.z --` — turn right
* L2 button
    * Rotate left: `twist.angular.z ++` — turn left
### 参考資料
[joy_node]に関する説明
* [joy](https://docs.ros.org/en/humble/p/joy/index.html)
* [ROS2でPS4のジョイスティック（DualShock4）を使ってみました](https://kanpapa.com/today/2022/09/ros2-joy-ps4-dualshock.html)

## 更新履歴
* 2025/01/15: [README.md を作成]
