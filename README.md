# rm_referee_ros2

RoboMaster 裁判系统的 ROS2 驱动，用于简化 ROS2 下和裁判系统的数据交互。

仓库的分支/tag 对应官方发布的协议文件版本，带 `legacy-` 前缀的 tag 为 26 赛季“串口协议”改为“通信协议”之前的版本，不带 `legacy-` 前缀的 tag 为 26 赛季改版为“通信协议”之后的版本。例如：

- `legacy-v1.7.0` 对应 《RoboMaster 裁判系统串口协议附录 V1.7.0（20241225）》。

- `v1.1.0` 对应 《RoboMaster 2026 机甲大师高校系列赛通信协议 V1.1.0（20251217）》。

`main`分支为最新开发版本，可能包含不稳定的改动。

## Build status

<table>
<tr>
<td>foxy</td>
<td rowspan=2>
    <a href="https://github.com/XDU-IRobot/rm_referee_ros2/actions/workflows/build_eol.yaml">
        <img src="https://github.com/XDU-IRobot/rm_referee_ros2/actions/workflows/build_eol.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>galactic</td>
</tr>
<tr>
<td>humble</td>
<td rowspan=4>
    <a href="https://github.com/XDU-IRobot/rm_referee_ros2/actions/workflows/build.yaml">
        <img src="https://github.com/XDU-IRobot/rm_referee_ros2/actions/workflows/build.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>iron</td>
</tr>
<tr>
<td>jazzy</td>
</tr>
<tr>
<td>rolling</td>
</tr>
</table>

## 使用方法

1. clone 本仓库到工作空间里，必须加 `--recursive` 参数以获取子模块：

```bash
git clone --recursive https://github.com/XDU-IRobot/rm_referee_ros2.git
```

2. `colcon build`编译，然后复制一份`rm_referee/launch/referee_node.launch.py`到你自己的项目里，按需修改里面的参数，运行即可。

裁判系统通过串口发送的数据会被封装成消息发布到对应话题上，反之可以通过请求`/rm_referee/tx`服务向裁判系统串口发送数据。

## Mock

`rm_referee_mock`包里提供了一些 Mock 组件，可以模拟裁判系统的数据发送行为，方便在没有真实裁判系统的情况下进行开发和测试。

具体的使用方法请参考 [`rm_referee_mock/README.md`](rm_referee_mock/README.md)。

## 话题列表

### 常规链路

| 话题名                            | 消息类型                                | 说明                 |
| --------------------------------- | --------------------------------------- | -------------------- |
| /rm_referee/buff                  | rm_referee_msgs/msg/Buff                | 机器人增益信息       |
| /rm_referee/dart_client_cmd       | rm_referee_msgs/msg/DartClientCmd       | 飞镖信息 1           |
| /rm_referee/dart_info             | rm_referee_msgs/msg/DartInfo            | 飞镖信息 2           |
| /rm_referee/event_data            | rm_referee_msgs/msg/EventData           | 比赛信息 1           |
| /rm_referee/game_result           | rm_referee_msgs/msg/GameResult          | 比赛结果             |
| /rm_referee/game_robot_hp         | rm_referee_msgs/msg/GameRobotHP         | 己方机器人血量       |
| /rm_referee/game_status           | rm_referee_msgs/msg/GameStatus          | 比赛信息 2           |
| /rm_referee/ground_robot_position | rm_referee_msgs/msg/GroundRobotPosition | 我方地面机器人位置   |
| /rm_referee/hurt_data             | rm_referee_msgs/msg/HurtData            | 扣血信息             |
| /rm_referee/power_heat_data       | rm_referee_msgs/msg/PowerHeatData       | 功率和枪口热量信息   |
| /rm_referee/projectile_allowance  | rm_referee_msgs/msg/ProjectileAllowance | 允许发弹量和剩余金币 |
| /rm_referee/radar_info            | rm_referee_msgs/msg/RadarInfo           | 雷达双倍易伤相关信息 |
| /rm_referee/radar_mark_data       | rm_referee_msgs/msg/RadarMarkData       | 对方机器人被标记进度 |
| /rm_referee/referee_warning       | rm_referee_msgs/msg/RefereeWarning      | 判罚信息             |
| /rm_referee/rfid_status           | rm_referee_msgs/msg/RFIDStatus          | RFID 检测信息        |
| /rm_referee/robot_pos             | rm_referee_msgs/msg/RobotPos            | 本机器人的位置和朝向 |
| /rm_referee/robot_status          | rm_referee_msgs/msg/RobotStatus         | 本机器人状态         |
| /rm_referee/sentry_info           | rm_referee_msgs/msg/SentryInfo          | 哨兵兑换信息         |
| /rm_referee/shoot_data            | rm_referee_msgs/msg/ShootData           | 弹丸信息             |
| /rm_referee/map_command           | rm_referee_msgs/msg/MapCommand          | 选手端小地图交互数据 |

### 图传链路

| 话题名                          | 消息类型                             | 说明                         |
| ------------------------------- | ------------------------------------ | ---------------------------- |
| /rm_referee/custom_robot_data   | rm_referee_msgs/msg/CustomRobotData  | 自定义控制器与机器人交互数据 |
| /rm_referee/robot_custom_data   | rm_referee_msgs/msg/RobotCustomData  | 自定义控制器接收机器人数据   |
| /rm_referee/robot_custom_data_2 | rm_referee_msgs/msg/RobotCustomData2 | 机器人发送给自定义客户端的数 |
| 据                              |
| /rm_referee/remote_control      | rm_referee_msgs/msg/RemoteControl    | 图传链路键鼠遥控数据         |

## 服务列表

| 服务名         | 服务类型               | 说明                   |
| -------------- | ---------------------- | ---------------------- |
| /rm_referee/tx | rm_referee_msgs/srv/Tx | 向裁判系统串口发送数据 |
