#!/usr/bin/env python3

from time import time

from rqt_gui_py.plugin import Plugin
# [新增] 引入 Qt 信号机制，解决多线程报错问题
from python_qt_binding.QtCore import QObject, pyqtSignal

from rm_referee_msgs.msg import (
    GameStatus, 
    GameRobotHP, 
    EventData, 
    RobotStatus, 
    ProjectileAllowance, 
    HurtData,
    SentryInfo 
)

try:
    from rm_referee_msgs.srv import Tx
except ImportError:
    # 兼容性处理：如果你还没有编译好 sentry_msgs，防止插件直接崩掉
    print("Warning: Could not import SetSentryPosture service.")
    Tx = None

from rm_referee_mock.match_control_widget import MatchControlWidget
from rm_referee_mock.publisher_pool import PublisherPool


# [新增] 创建一个用于跨线程通信的桥接类
class WorkerSignals(QObject):
    # 定义信号：(姿态值, 是否激活)
    update_sentry_signal = pyqtSignal(int, int)


class MatchControlPlugin(Plugin):
    """RQt Plugin wrapper for MatchControlWidget"""

    def __init__(self, context):
        super(MatchControlPlugin, self).__init__(context)
        self.setObjectName("MatchControlPlugin")

        self._widget = MatchControlWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()))

        context.add_widget(self._widget)
        self._node = context.node
        self._publisher_pool = PublisherPool(self._node)

        # [关键] 初始化信号桥接，绑定 UI 更新函数
        # 这样可以将 ROS 线程收到的数据安全地传给主线程进行 UI 更新
        self.signals = WorkerSignals()
        self.signals.update_sentry_signal.connect(self._ui_update_callback)

        # === Service Server 初始化 ===
        self.received_sentry_mode = 0
        if Tx:
            self._srv = self._node.create_service(
                Tx, 
                '/rm_referee/tx', 
                self._handle_sentry_cmd
            )
            
        self._timer = self._node.create_timer(0.1, self._timer_callback)

    def _handle_sentry_cmd(self, request, response):
        """
        处理收到的 Service 请求 (解析 C++ 发来的原始协议包)
        协议包结构: Header(5) + CmdID(2) + [SubHeader(6) + Content(4)] + CRC16(2) = 19 bytes
        关键数据在 Content (4 bytes)，位于 offset 13 (5+2+6)
        """
        raw_data = request.data
        data_len = len(raw_data)

        # [调试] 收到原始数据，证明 Service 链路是通的
        self._node.get_logger().info(f"[Service-Rx] 收到请求, 长度: {data_len} 字节")
        
        # 简单校验长度 (至少要有19字节)
        if data_len < 19:
            self._node.get_logger().warn(f"[Service-Error] 数据包长度不足! 收到: {data_len}, 期望: >=19")
            response.ok = False
            return response

        # --- 解析关键数据 ---
        # 0x0120 的数据在 offset 13 开始的 4 个字节 (uint32)
        # 我们需要解析第 3 个字节 (index 15)，因为它包含了 Bit 16-23
        # Bit 21-22: 姿态
        # Bit 23: 确认激活
        
        try:
            target_byte = raw_data[15] # 获取第16个字节 (包含 Bit 16-23)
            
            # 解析姿态 (Bit 21-22 -> 在该字节中是 Bit 5-6)
            posture_val = (target_byte >> 5) & 0x03
            
            # 解析激活确认 (Bit 23 -> 在该字节中是 Bit 7)
            activate_confirm = (target_byte >> 7) & 0x01
            
            # [调试] 解析结果
            self._node.get_logger().info(
                f"[Service-Parse] 解析成功 -> 姿态: {posture_val}, 激活请求: {activate_confirm}"
            )

            # 1. 更新内部状态 (用于回显发布)
            self.received_sentry_mode = posture_val
            
            # 2. 发送信号给主线程 (解决 QObject::startTimer 报错的关键)
            # 千万不要在这里直接调用 self._widget 的方法！
            self.signals.update_sentry_signal.emit(posture_val, activate_confirm)
            
            # 3. 填充 Response Header (规范做法)
            response.header.stamp = self._node.get_clock().now().to_msg()
            response.header.frame_id = "referee_mock"

            # 4. 返回成功 (注意字段名是 ok)
            response.ok = True 

        except Exception as e:
            self._node.get_logger().error(f"[Service-Exception] 解析或处理错误: {str(e)}")
            response.ok = False

        return response

    def _ui_update_callback(self, posture_val, activate_confirm):
        """
        [主线程] 这个函数由 Qt 信号触发，专门负责更新 UI
        """
        try:
            # 更新 UI 显示 (Label 变色/文字改变)
            self._widget.update_sentry_echo(posture_val)
            
            # 处理激活逻辑
            if activate_confirm == 1:
                self._node.get_logger().info("[UI-Logic] 收到激活请求，调用 Widget 逻辑...")
                
                # 调用 Widget 的逻辑 (检查 flag, 设置 timer, 取消勾选)
                success = self._widget.confirm_activation()
                
                if success:
                    self._node.get_logger().info(">>> [UI-Success] 激活成功！倒计时开始")
                else:
                    self._node.get_logger().warn(">>> [UI-Fail] 激活被拒绝！(无权限/冷却中)")
            
        except Exception as e:
            self._node.get_logger().error(f"[UI-Exception] UI 更新出错: {str(e)}")

    def _timer_callback(self):
        """Timer callback to publish match status periodically"""
        # Get current status from widget
        status = self._widget.get_game_status()
        topic_prefix = self._widget.get_topic_prefix()

        current_time_msg = self._node.get_clock().now().to_msg()

        # Publish GameStatus message
        game_status_msg = GameStatus()
        game_status_msg.header.stamp = current_time_msg
        game_status_msg.header.frame_id = "referee_system"
        game_status_msg.game_type = int(status["game_type"])
        game_status_msg.game_progress = int(status["game_progress"])
        game_status_msg.stage_remain_time = int(status["stage_remain_time"])
        game_status_msg.sync_timestamp = int(time() * 1000)

        self._publisher_pool.publish(
            f"{topic_prefix}/game_status", GameStatus, game_status_msg)

        # Publish GameRobotHP message
        robot_hp_msg = GameRobotHP()
        robot_hp_msg.header.stamp = current_time_msg
        robot_hp_msg.header.frame_id = "referee_system"
        robot_hp_msg.ally_1_robot_hp = int(status["robot_hp"]["hero"])
        robot_hp_msg.ally_2_robot_hp = int(status["robot_hp"]["engineer"])
        robot_hp_msg.ally_3_robot_hp = int(status["robot_hp"]["infantry_3"])
        robot_hp_msg.ally_4_robot_hp = int(status["robot_hp"]["infantry_4"])
        robot_hp_msg.reserved = 0
        robot_hp_msg.ally_7_robot_hp = int(status["robot_hp"]["sentry"])
        robot_hp_msg.ally_outpost_hp = int(status["robot_hp"]["outpost"])
        robot_hp_msg.ally_base_hp = int(status["robot_hp"]["base"])

        self._publisher_pool.publish(
            f"{topic_prefix}/game_robot_hp", GameRobotHP, robot_hp_msg)
        
        # [新增] Publish EventData 
        event_msg = EventData()
        event_msg.header.stamp = current_time_msg
        event_msg.event_data = int(status["event_data"]) 
        self._publisher_pool.publish(
            f"{topic_prefix}/event_data", EventData, event_msg)
            
        # [新增] Publish RobotStatus 
        robot_status_msg = RobotStatus()
        robot_status_msg.robot_id = int(status["robot_status"]["id"])
        robot_status_msg.robot_level = 1
        robot_status_msg.current_hp = int(status["robot_status"]["current_hp"])
        robot_status_msg.maximum_hp = int(status["robot_status"]["max_hp"])
        robot_status_msg.shooter_barrel_cooling_value = 0
        robot_status_msg.shooter_barrel_heat_limit = 100
        robot_status_msg.chassis_power_limit = 120
        # 假定功率管理都开启
        robot_status_msg.power_management_gimbal_output = True
        robot_status_msg.power_management_chassis_output = True
        robot_status_msg.power_management_shooter_output = True
        self._publisher_pool.publish(
            f"{topic_prefix}/robot_status", RobotStatus, robot_status_msg)

        # [新增] Publish ProjectileAllowance 
        ammo_msg = ProjectileAllowance()
        ammo_msg.projectile_allowance_17mm = int(status["robot_status"]["ammo"])
        self._publisher_pool.publish(
            f"{topic_prefix}/projectile_allowance", ProjectileAllowance, ammo_msg)
            
        # [新增] Publish HurtData (Triggered)
        if status["hurt_data"]["trigger"]:
            hurt_msg = HurtData()
            hurt_msg.armor_id = int(status["hurt_data"]["armor_id"])
            hurt_msg.hp_deduction_reason = 0 # 默认为0，装甲板受击
            self._publisher_pool.publish(
                f"{topic_prefix}/hurt_data", HurtData, hurt_msg)

        # [新增] Publish SentryInfo (Echo Service Result)
        sentry_info_msg = SentryInfo()
        sentry_info_msg.header.stamp = current_time_msg
        
        # 获取数据: 姿态来自 Service 回调, 可激活标志来自 Widget
        posture_bits = (self.received_sentry_mode & 0x03)  
        
        can_activate = 0
        if hasattr(self._widget, 'checkBox_can_activate_rune'):
            if self._widget.checkBox_can_activate_rune.isChecked():
                can_activate = 1

        # 位运算拼接: 姿态左移 12 位 | 打符左移 14 位
        packed_info_2 = (posture_bits << 12) | (can_activate << 14)
        sentry_info_msg.sentry_info_2 = packed_info_2
        
        self._publisher_pool.publish(
            f"{topic_prefix}/sentry_info", SentryInfo, sentry_info_msg)

    def shutdown_plugin(self):
        """Clean up when plugin is closed"""
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def save_settings(self, plugin_settings, instance_settings):
        """Save the plugin settings"""
        # TODO: Save widget state, if needed
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the plugin settings"""
        # TODO: Restore widget state, if needed
        pass