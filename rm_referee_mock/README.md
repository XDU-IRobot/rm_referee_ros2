# rm_referee_mock

这是 `rm_referee_ros2` 仓库下的一个子模块，提供了一些用于测试的 Mock 组件，可以模拟裁判系统的数据发送行为，方便在没有真实裁判系统的情况下进行开发和测试。

## keyboard_publisher

`keyboard_publisher` 模拟图传链路的键鼠数据发送功能。它读取本地键盘输入，并将输入的数据封装成 `rm_referee_msgs/RemoteControl` 消息发布到指定话题上。

### 使用方法

运行节点：

```bash
ros2 run rm_referee_mock keyboard_publisher --ros-args -p publish_topic:=/rm_referee/mock/remote_control -p publish_rate:=20
```

运行后会弹出一个窗口，聚焦窗口后按键盘即可。

### 参数

| 参数名          | 说明                      | 默认值                            |
| --------------- | ------------------------- | --------------------------------- |
| `publish_topic` | 发布数据的 ROS 话题名     | `/rm_referee/mock/remote_control` |
| `publish_rate`  | 发布数据的频率，单位为 Hz | `20.0`                            |

## dart_client

`dart_client` 模拟云台手客户端的飞镖发射命令。它读取本地键盘输入，并根据输入的命令构造 `rm_referee_msgs/DartClientCmd` 消息发布到指定话题上。

### 使用方法

运行节点：

```bash
ros2 run rm_referee_mock dart_client --ros-args -p publish_topic:=/rm_referee/mock/dart_client_cmd -p publish_rate:=1
```

运行后会在终端显示按键说明。

### 参数

| 参数名          | 说明                      | 默认值                             |
| --------------- | ------------------------- | ---------------------------------- |
| `publish_topic` | 发布数据的 ROS 话题名     | `/rm_referee/mock/dart_client_cmd` |
| `publish_rate`  | 发布数据的频率，单位为 Hz | `10.0`                             |
