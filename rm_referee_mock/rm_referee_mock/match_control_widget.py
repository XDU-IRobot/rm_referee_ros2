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

        # 创建定时器用于倒计时
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.update_time)

        # 初始化控件
        self.init_ui()

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
        self.spinBox_hp_1.setMaximum(600)    # 英雄
        self.spinBox_hp_2.setMaximum(400)  # 工程
        self.spinBox_hp_3.setMaximum(250)  # 步兵
        self.spinBox_hp_4.setMaximum(250)  # 步兵
        self.spinBox_hp_7.setMaximum(600)  # 哨兵
        self.spinBox_hp_outpost.setMaximum(1500)  # 前哨站
        self.spinBox_hp_base.setMaximum(5000)  # 基地

        # 设置初始HP值
        self.spinBox_hp_1.setValue(600)
        self.spinBox_hp_2.setValue(400)
        self.spinBox_hp_3.setValue(250)
        self.spinBox_hp_4.setValue(250)
        self.spinBox_hp_7.setValue(600)
        self.spinBox_hp_outpost.setValue(1500)
        self.spinBox_hp_base.setValue(5000)

        # 连接信号和槽
        self.pushButton_playpause.clicked.connect(self.on_play_pause_clicked)

        # 将普通 LineEdit 替换为 ConfirmedLineEdit
        self._replace_lineedit_with_confirmed("lineEdit_time", "0:00")
        self._replace_lineedit_with_confirmed(
            "lineEdit_topic_prefix", "/rm_referee/mock")
        self._replace_lineedit_with_confirmed("lineEdit_dart_time", "0:00")

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

    def update_time(self):
        """更新比赛倒计时"""
        current_time = self.lineEdit_time.get_confirmed_text()

        # 解析时间格式 MM:SS
        try:
            parts = current_time.split(":")
            if len(parts) == 2:
                minutes = int(parts[0])
                seconds = int(parts[1])

                # 检查是否已经到达 7:00
                if minutes == 7 and seconds == 0:
                    self.countdown_timer.stop()
                    self.pushButton_playpause.setText("▶")
                    self.lineEdit_time.setReadOnly(False)
                    return

                # 递增时间
                seconds += 1
                if seconds >= 60:
                    seconds = 0
                    minutes += 1

                # 更新显示和确认值
                new_time = f"{minutes}:{seconds:02d}"
                self.lineEdit_time.set_confirmed_text(new_time)
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
            "field_status": {
                "small_rune": self.comboBox_small_rune.currentText(),
                "big_rune": self.comboBox_big_rune.currentText(),
                "central_highland": self.comboBox_central_highland.currentText(),
                "trapezoid_highland": self.checkBox_4.isChecked(),
                "resource_zone": self.checkBox.isChecked(),
                "supply_zone": self.checkBox_2.isChecked(),
                "supply_zone_ul": self.checkBox_3.isChecked(),
                "central_buff": self.comboBox_central_buff.currentText(),
                "fortress_buff": self.comboBox_fortress_buff.currentText(),
                "base_buff": self.checkBox_base_buff.isChecked(),
                "dart_target": self.comboBox_dart_target.currentText(),
                "dart_hit_time": self.lineEdit_dart_time.get_confirmed_text(),
            }
        }
        return status


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MatchControlWidget()
    window.setWindowTitle("RM Match Server Sim")
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
