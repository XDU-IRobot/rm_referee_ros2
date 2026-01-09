# rm_referee_mock

这是 `rm_referee_ros2` 仓库下的一个子模块，提供了一些用于测试的 Mock 组件，可以模拟裁判系统的数据发送行为，方便在没有真实裁判系统的情况下进行开发和测试。

目前为止所有的 Mock 组件均以 rqt 插件的形式实现，使用方法为编译并 source 工作空间后，在 rqt 中启动对应的插件即可。

## Keyboard Publisher

![keyboard_publisher_rqt](../docs/keyboard_publisher_rqt.jpg)

模拟图传链路的键鼠数据发送功能。Keyboard Publisher 会读取键盘输入，并将输入的数据封装成 `rm_referee_msgs/RemoteControl` 消息发布到指定话题上。

## Dart Client

![dart_client_rqt](../docs/dart_client_rqt.jpg)

模拟比赛服务器下发的飞镖相关信息和云台手客户端的操作，包括：

- `0x0105` `rm_referee_msgs/DartInfo`：飞镖发射相关数据（固定以 3Hz 频率发送）
- `0x020A` `rm_referee_msgs/DartClientCmd`：飞镖选手端指令数据（固定以 3Hz 频率发送）

## Match Control [WIP!]

![match_control_rqt](../docs/match_control_rqt.jpg)

手动控制比赛数据发布。可以控制发布的数据包括：

- `0x0001` `rm_referee_msgs/GameStatus`：比赛状态
- `0x0003` `rm_referee_msgs/GameRobotHP`：己方机器人血量
- `0x0101` `rm_referee_msgs/EventData`：场地事件数据

如果有需要，可以扩展更多比赛状态的控制项。
