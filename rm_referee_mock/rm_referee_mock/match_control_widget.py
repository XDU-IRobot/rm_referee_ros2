#!/usr/bin/env python3

import sys
from os import path

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
from rm_referee_mock.widget_confirmed_line_edit import ConfirmedLineEdit

from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = get_package_share_directory("rm_referee_mock")


class MatchControlWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        
        # 加载 UI 文件
        ui_file = path.join(PACKAGE_SHARE, "assets", "match_control.ui")
        uic.loadUi(ui_file, self)
        
        self.setWindowTitle("Match Control")

        # 创建定时器用于倒计时
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.update_time)

        # 初始化控件
        self.init_ui()

        # 内部变量用于存储 HurtData (点击按钮时才更新)
        self.current_hurt_armor_id = 0
        self.trigger_hurt = False

        # === 初始化能量机关逻辑变量 ===
        # 是否允许激活（对应 sentry_info Bit 14）
        self.sentry_can_activate_flag = False
        # 记录当前获得权限的是哪种符 (0:无, 1:小符, 2:大符)
        self.authorized_rune_type = 0
        # 正在激活的倒计时器
        self.rune_activating_timer = 0
        # 机会计数器
        self.small_rune_chances = 0 # 小符剩余次数
        self.big_rune_chances = 0   # 大符剩余次数
        
        # 记录上一次的时间，防止同一秒钟加多次
        self.last_remain_seconds = -1

    def init_ui(self):
        """初始化 UI 控件的默认值和选项"""
        # 设置比赛类型选项 (索引0不使用，从1开始)
        self.comboBox_game_type.addItems([
            "[0] 未定义",
            "[1] RoboMaster 机甲大师超级对抗赛",
            "[2] RoboMaster 机甲大师高校单项赛",
            "[3] ICRA RoboMaster高校人工智能挑战赛",
            "[4] RoboMaster机甲大师高校联盟赛3V3对抗",
            "[5] RoboMaster 机甲大师高校联盟赛步兵对抗"
        ])
        self.comboBox_game_type.setCurrentIndex(1)  # 默认选择RMUC

        # 设置比赛阶段选项
        self.comboBox_game_stage.addItems([
            "[0] 未开始比赛",
            "[1] 准备阶段",
            "[2] 十五秒裁判系统自检阶段",
            "[3] 五秒倒计时",
            "[4] 比赛中",
            "[5] 比赛结算中"
        ])

        # 设置能量机关状态选项
        # 0:未激活, 1:激活中, 2:已激活
        energy_status = ["未激活", "激活中", "已激活"]
        self.comboBox_small_rune.addItems(energy_status)  # 小能量机关
        self.comboBox_big_rune.addItems(energy_status)  # 大能量机关

        # 设置高地占领状态
        occupy_status = ["无占领", "己方占领", "敌方占领", "争夺中"]
        self.comboBox_central_highland.addItems(occupy_status)  # 己方中央高地

        # 设置增益点状态
        buff_status = ["未激活", "激活中", "己方占领", "敌方占领"]
        self.comboBox_central_buff.addItems(buff_status)  # 中心增益点
        self.comboBox_fortress_buff.addItems(buff_status)  # 己方堡垒增益点
        self.comboBox_outpost_buff.addItems(buff_status)  # 己方前哨站增益点

        # 设置飞镖击中目标
        dart_targets = ["未击中", "前哨站", "基地"]
        self.comboBox_dart_target.addItems(dart_targets)

        # 设置HP默认值
        self.spinBox_hp_1.setMaximum(200)    # 英雄   近战优先1级上限血量200 ，远程优先1级上限血量150
        self.spinBox_hp_2.setMaximum(250)  # 工程
        self.spinBox_hp_3.setMaximum(250)  # 步兵
        self.spinBox_hp_4.setMaximum(250)  # 步兵
        self.spinBox_hp_7.setMaximum(400)  # 哨兵
        self.spinBox_hp_outpost.setMaximum(1500)  # 前哨站
        self.spinBox_hp_base.setMaximum(5000)  # 基地

        # 设置初始HP值
        self.spinBox_hp_1.setValue(200)
        self.spinBox_hp_2.setValue(250)
        self.spinBox_hp_3.setValue(250)
        self.spinBox_hp_4.setValue(250)
        self.spinBox_hp_7.setValue(400)
        self.spinBox_hp_outpost.setValue(1500)
        self.spinBox_hp_base.setValue(5000)

        # 连接信号和槽
        self.pushButton_playpause.clicked.connect(self.on_play_pause_clicked)

        # 伤害发送按钮
        if hasattr(self, 'btn_send_hurt'):
            self.btn_send_hurt.clicked.connect(self.on_hurt_button_clicked)

        # 将普通 LineEdit 替换为 ConfirmedLineEdit
        self._replace_lineedit_with_confirmed("lineEdit_time", "7:00")
        self._replace_lineedit_with_confirmed(
            "lineEdit_topic_prefix", "/rm_referee")
        self._replace_lineedit_with_confirmed("lineEdit_dart_time", "0:00")

        # 确保新控件有默认值 (防止直接运行旧 UI 报错)  
        if not hasattr(self, 'spinBox_ammo'): self.spinBox_ammo = QtWidgets.QSpinBox()
        if not hasattr(self, 'spinBox_robot_id'): self.spinBox_robot_id = QtWidgets.QSpinBox()

    def _replace_lineedit_with_confirmed(self, object_name, default_text):
        """替换一个LineEdit为ConfirmedLineEdit"""
        old_widget = getattr(self, object_name, None)
        if old_widget is None:
            print(f"Warning: {object_name} not found")
            return

        # 获取父控件和布局信息
        parent = old_widget.parent()
        if parent is None:
            print(f"Warning: {object_name} has no parent")
            return

        # 创建新的ConfirmedLineEdit
        new_widget = ConfirmedLineEdit(parent)
        new_widget.setObjectName(object_name)
        new_widget.setText(default_text)
        new_widget.set_confirmed_text(default_text)

        # 复制样式和属性
        new_widget.setEnabled(old_widget.isEnabled())
        new_widget.setReadOnly(old_widget.isReadOnly())
        new_widget.setPlaceholderText(old_widget.placeholderText())
        new_widget.setSizePolicy(old_widget.sizePolicy())
        new_widget.setMinimumSize(old_widget.minimumSize())
        new_widget.setMaximumSize(old_widget.maximumSize())

        # 从布局中替换
        layout = parent.layout()
        if layout:
            for i in range(layout.count()):
                if layout.itemAt(i).widget() == old_widget:
                    layout.removeWidget(old_widget)
                    layout.insertWidget(i, new_widget)
                    break
        else:
            # 如果没有布局，使用geometry
            new_widget.setGeometry(old_widget.geometry())

        # 隐藏并删除旧控件
        old_widget.hide()
        old_widget.deleteLater()

        # 更新引用
        setattr(self, object_name, new_widget)
        new_widget.show()

    def on_play_pause_clicked(self):
        """处理播放/暂停按钮点击事件"""
        if self.pushButton_playpause.text() == "▶":
            # 开始比赛
            self.pushButton_playpause.setText("⏸")
            self.lineEdit_time.setReadOnly(True)
            # 启动倒计时，每1000ms（1秒）触发一次
            self.countdown_timer.start(1000)
        else:
            # 暂停比赛
            self.pushButton_playpause.setText("▶")
            self.lineEdit_time.setReadOnly(False)
            # 停止倒计时
            self.countdown_timer.stop()

    def on_hurt_button_clicked(self):
        """点击发送伤害"""
        self.trigger_hurt = True
        self.current_hurt_armor_id = self.spinBox_hurt_armor_id.value()
        
        
    def _reset_hurt_trigger(self):
        self.trigger_hurt = False

    def update_time(self):
        """更新比赛倒计时 """
        current_time = self.lineEdit_time.get_confirmed_text()

        # 解析时间格式 MM:SS
        try:
            parts = current_time.split(":")
            if len(parts) == 2:
                minutes = int(parts[0])
                seconds = int(parts[1])

                # 检查是否已经到达 0:00
                if minutes == 0 and seconds == 0:
                    self.countdown_timer.stop()
                    self.pushButton_playpause.setText("▶")
                    self.lineEdit_time.setReadOnly(False)
                    return

                # 递减时间
                if seconds > 0:
                    seconds -= 1
                else:
                    if minutes > 0:
                        minutes -= 1
                        seconds = 59
                    else:
                        # 应该不会走到这里，因为上面 check 过了
                        seconds = 0

                # 更新显示和确认值
                new_time = f"{minutes}:{seconds:02d}"
                self.lineEdit_time.set_confirmed_text(new_time)

                # 每一秒都检查是否到了刷新符的时间点
                # 计算当前剩余总秒数
                total_seconds = minutes * 60 + seconds
                self._check_rune_refresh(total_seconds)

        except (ValueError, IndexError):
            # 如果时间格式不正确，停止计时器
            self.countdown_timer.stop()
            self.pushButton_playpause.setText("▶")
            self.lineEdit_time.setReadOnly(False)

    def _parse_time_to_seconds(self, time_str=None):
        """解析时间字符串 MM:SS 转换为秒数"""
        # 如果没有提供时间字符串，使用已确认的值
        if time_str is None:
            time_str = self.lineEdit_time.get_confirmed_text()
        try:
            parts = time_str.strip().split(":")
            if len(parts) == 2:
                minutes = int(parts[0])
                seconds = int(parts[1])
                return minutes * 60 + seconds
        except (ValueError, IndexError):
            pass
        return 0

    def get_topic_prefix(self):
        """获取话题前缀"""
        prefix = self.lineEdit_topic_prefix.get_confirmed_text()
        # 确保前缀以/开头，但不以/结尾
        if prefix and not prefix.startswith("/"):
            prefix = "/" + prefix
        if prefix.endswith("/"):
            prefix = prefix.rstrip("/")
        return prefix if prefix else ""
    
    def get_event_data_value(self):
        """
            计算 EventData 的位图值
            增加数值映射，确保发出的状态符合行为树/裁判系统协议预期
        """
        # C++ 解析逻辑:
        # small_rune = (data >> 3) & 0x03
        # large_rune = (data >> 5) & 0x03
        
        # --- 内部函数定义 ---
        def map_status_val(ui_index):
            if ui_index == 0: return 0
            if ui_index == 1: return 2  # 映射: 激活中 -> 2
            if ui_index == 2: return 1  # 映射: 已激活 -> 1
            return 0
        # ------------------

        # 1. 获取 UI 原始索引
        small_idx = self.comboBox_small_rune.currentIndex()
        large_idx = self.comboBox_big_rune.currentIndex()
        
        # 2. 转换数值
        small_val = map_status_val(small_idx)
        large_val = map_status_val(large_idx)
        
        # 3. [核心修复点] 必须先初始化变量！
        event_val = 0
        
        # 4. 位运算拼接 (Small 在 Bit 3-4, Large 在 Bit 5-6)
        event_val |= (small_val & 0x03) << 3
        event_val |= (large_val & 0x03) << 5

        # 可以在这里添加其他事件（补给站占领等），目前只加了能量机关
        return event_val

    def get_game_status(self):
        """获取当前比赛状态"""
        status = {
            "game_type": self.comboBox_game_type.currentIndex(),
            "game_progress": self.comboBox_game_stage.currentIndex(),
            "stage_remain_time": self._parse_time_to_seconds(self.lineEdit_time.text()),
            "robot_hp": {
                "hero": self.spinBox_hp_1.value(),
                "engineer": self.spinBox_hp_2.value(),
                "infantry_3": self.spinBox_hp_3.value(),
                "infantry_4": self.spinBox_hp_4.value(),
                "sentry": self.spinBox_hp_7.value(),
                "outpost": self.spinBox_hp_outpost.value(),
                "base": self.spinBox_hp_base.value(),
            },
            # 详细机器人状态
            "robot_status": {
                "id": self.spinBox_robot_id.value(),
                "current_hp": self.spinBox_hp_7.value(), # 复用哨兵血量
                "max_hp": 600,
                "ammo": self.spinBox_ammo.value()
            },
            # 事件数据
            "event_data": self.get_event_data_value(),
            # 伤害数据
            "hurt_data": {
                "armor_id": self.current_hurt_armor_id if self.trigger_hurt else 0,
                # 如果 trigger 为 false, 发 0 或者不发(plugin处理)
                "trigger": self.trigger_hurt 
            },
            # === 新增：传出可激活标志，用于 sentry_info Bit 14 ===
            "sentry_can_activate": self.sentry_can_activate_flag
        }
        if self.trigger_hurt:
            self.trigger_hurt = False

        return status

    def update_sentry_echo(self, mode):
        """更新 UI 显示收到的指令"""
        if hasattr(self, 'label_sentry_mode'):
            # 简单映射模式名
            names = {0: "无效", 1: "进攻", 2: "防御", 3: "移动"}
            txt = names.get(mode, str(mode))
            self.label_sentry_mode.setText(f"Mode: {txt}")
            # 可以变个色提示更新
            self.label_sentry_mode.setStyleSheet("color: red; font-weight: bold;")
            QTimer.singleShot(500, lambda: self.label_sentry_mode.setStyleSheet("color: blue; font-weight: bold;"))    

    def _check_rune_refresh(self, remain_seconds):
        """
        根据比赛剩余时间，自动刷新能量机关状态，增加激活机会次数
        流程：时间到了 -> 设置 sentry_info Bit 14 = 1 (可激活) -> 等待 Service 确认 -> 变为激活中
        """
        
        # 防止同一秒执行多次
        if remain_seconds == self.last_remain_seconds:
            return
        self.last_remain_seconds = remain_seconds

        # === 1. 发放激活机会 (增加计数) ===
        
        # 小符时间点: 7:00(420s), 5:30(330s)
        if remain_seconds in [420, 330]:
            self.small_rune_chances += 1
            print(f"[Rule] 时间 {remain_seconds}s: 获得1次小能量机关激活机会 (当前累积: {self.small_rune_chances})")

        # 大符时间点: 4:00(240s), 2:45(165s), 1:30(90s)
        if remain_seconds in [240, 165, 90]:
            self.big_rune_chances += 1
            print(f"[Rule] 时间 {remain_seconds}s: 获得1次大能量机关激活机会 (当前累积: {self.big_rune_chances})")

        # === 2. 更新 sentry_info 的权限位 (Bit 14) ===

        current_small_status = self.comboBox_small_rune.currentIndex() # 0:未激活
        current_big_status = self.comboBox_big_rune.currentIndex()     # 0:未激活
        
        can_activate = False
        
        # 有小符机会 且 小符未激活
        if self.small_rune_chances > 0 and current_small_status == 0:
            can_activate = True
            
        # 有大符机会 且 大符未激活
        if self.big_rune_chances > 0 and current_big_status == 0:
            can_activate = True
            
        # 同步给 flag，用于发布给机器人
        self.sentry_can_activate_flag = can_activate
        
        # 同步 UI 勾选框 (视觉反馈)
        if hasattr(self, 'checkBox_can_activate_rune'):
            # 如果人工没介入，自动更新
            if self.sentry_can_activate_flag:
                self.checkBox_can_activate_rune.setChecked(True)

        # === 3. 倒计时逻辑  ===
        # 如果处于"正在激活"(Index=1)，处理20s倒计时
        if current_small_status == 1 or current_big_status == 1:
            if self.rune_activating_timer > 0:
                self.rune_activating_timer -= 1
                if self.rune_activating_timer == 0:
                    print("[Timeout] 激活时间窗口结束，重置状态")
                    if current_small_status == 1: self.comboBox_small_rune.setCurrentIndex(0)
                    if current_big_status == 1: self.comboBox_big_rune.setCurrentIndex(0)


    def confirm_activation(self):
        """
        哨兵发来了确认指令，尝试进入'激活中'状态，消耗累积次数
        供 Plugin 的 Service 回调调用
        """
        # 优先检查 UI 强制勾选 (方便调试)
        ui_checked = False
        if hasattr(self, 'checkBox_can_activate_rune'):
            ui_checked = self.checkBox_can_activate_rune.isChecked()

        # 如果 Flag 为 True (来自自动逻辑) 或 UI 被勾选
        if self.sentry_can_activate_flag or ui_checked:
            
            # === 判定机器人想打哪个符 ===
            # 这里做一个简化的假设：机器人通常先打小符，再打大符。
            # 也可以根据当前比赛时间或机器人发来的姿态进一步判断，但这里简化处理。
            
            activated_type = 0 # 1:小符, 2:大符
            
            current_small_status = self.comboBox_small_rune.currentIndex()
            current_big_status = self.comboBox_big_rune.currentIndex()

            # 逻辑：如果小符有次数且没激活 -> 激活小符
            if self.small_rune_chances > 0 and current_small_status == 0:
                activated_type = 1
                self.small_rune_chances -= 1 # 扣除次数
                print(f">>> [Success] 消耗1次小符机会 (剩余: {self.small_rune_chances})")
                
            # 否则：如果大符有次数且没激活 -> 激活大符
            elif self.big_rune_chances > 0 and current_big_status == 0:
                activated_type = 2
                self.big_rune_chances -= 1 # 扣除次数
                print(f">>> [Success] 消耗1次大符机会 (剩余: {self.big_rune_chances})")
                
            # 特殊情况：如果是 UI 强制勾选但次数为0 (调试模式)
            elif ui_checked:
                 if current_small_status == 0: activated_type = 1
                 elif current_big_status == 0: activated_type = 2
                 print(">>> [Debug] UI强制激活")

            # === 执行激活 ===
            if activated_type > 0:
                self.rune_activating_timer = 20 # 开启20秒窗口
                
                if activated_type == 1:
                    self.comboBox_small_rune.setCurrentIndex(1) # 设为: 正在激活
                elif activated_type == 2:
                    self.comboBox_big_rune.setCurrentIndex(1) # 设为: 正在激活
                
                # 激活成功后，如果手里没有剩余次数了，就把勾选框取消
                # 如果还有次数（比如累积了2次），就保持勾选，允许机器人接着打
                total_left = self.small_rune_chances + self.big_rune_chances
                if total_left == 0:
                    self.sentry_can_activate_flag = False
                    if hasattr(self, 'checkBox_can_activate_rune'):
                        self.checkBox_can_activate_rune.setChecked(False)
                
                return True
            else:
                print(">>> [Fail] 虽然有权限，但没有可激活的目标 (可能都已激活)")
                return False
        else:
            print(">>> [Fail] 激活拒绝：无累积次数")
            return False
        
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MatchControlWidget()
    window.setWindowTitle("RM Match Server Sim")
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

