#!/usr/bin/env python3
import sys
from os import path
from PyQt5 import QtWidgets, uic

from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = get_package_share_directory('rm_referee_mock')


class MatchControlWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # 加载 UI 文件
        ui_file = path.join(PACKAGE_SHARE, "assets", "match_control.ui")
        uic.loadUi(ui_file, self)

        # 初始化控件
        self.init_ui()

    def init_ui(self):
        """初始化 UI 控件的默认值和选项"""
        # 设置比赛类型选项
        self.comboBox.addItems(["RMUC", "RMUL", "RMUA", "RMUT"])

        # 设置比赛阶段选项
        self.comboBox_2.addItems(
            ["未开始", "准备阶段", "自检阶段", "5s倒计时", "比赛中", "比赛结算"])

        # 设置能量机关状态选项
        energy_status = ["未激活", "激活中", "已激活"]
        self.comboBox_3.addItems(energy_status)  # 小能量机关
        self.comboBox_4.addItems(energy_status)  # 大能量机关

        # 设置高地占领状态
        occupy_status = ["无占领", "己方占领", "敌方占领", "争夺中"]
        self.comboBox_5.addItems(occupy_status)  # 己方中央高地

        # 设置增益点状态
        buff_status = ["未激活", "激活中", "己方占领", "敌方占领"]
        self.comboBox_7.addItems(buff_status)  # 中心增益点
        self.comboBox_8.addItems(buff_status)  # 己方堡垒增益点
        self.comboBox_9.addItems(buff_status)  # 环形增益点

        # 设置飞镖击中目标
        dart_targets = ["未击中", "前哨站", "基地"]
        self.comboBox_6.addItems(dart_targets)

        # 设置HP默认值
        self.spinBox.setMaximum(600)    # 英雄
        self.spinBox_2.setMaximum(400)  # 工程
        self.spinBox_3.setMaximum(250)  # 步兵
        self.spinBox_4.setMaximum(250)  # 步兵
        self.spinBox_5.setMaximum(600)  # 哨兵
        self.spinBox_6.setMaximum(1500)  # 前哨站
        self.spinBox_7.setMaximum(5000)  # 基地

        # 设置初始HP值
        self.spinBox.setValue(600)
        self.spinBox_2.setValue(400)
        self.spinBox_3.setValue(250)
        self.spinBox_4.setValue(250)
        self.spinBox_5.setValue(600)
        self.spinBox_6.setValue(1500)
        self.spinBox_7.setValue(5000)

        # 连接信号和槽
        self.pushButton.clicked.connect(self.on_play_pause_clicked)

    def on_play_pause_clicked(self):
        """处理播放/暂停按钮点击事件"""
        if self.pushButton.text() == "▶":
            self.pushButton.setText("⏸")
            print("比赛开始")
            # TODO: 实现比赛开始逻辑
        else:
            self.pushButton.setText("▶")
            print("比赛暂停")
            # TODO: 实现比赛暂停逻辑

    def get_game_status(self):
        """获取当前比赛状态"""
        status = {
            'game_type': self.comboBox.currentText(),
            'game_stage': self.comboBox_2.currentText(),
            'time': self.lineEdit.text(),
            'robot_hp': {
                'hero': self.spinBox.value(),
                'engineer': self.spinBox_2.value(),
                'infantry_3': self.spinBox_3.value(),
                'infantry_4': self.spinBox_4.value(),
                'sentry': self.spinBox_5.value(),
                'outpost': self.spinBox_6.value(),
                'base': self.spinBox_7.value(),
            },
            'field_status': {
                'small_energy': self.comboBox_3.currentText(),
                'large_energy': self.comboBox_4.currentText(),
                'central_highland': self.comboBox_5.currentText(),
                'trapezoid_highland': self.checkBox_4.isChecked(),
                'resource_zone': self.checkBox.isChecked(),
                'supply_zone': self.checkBox_2.isChecked(),
                'supply_zone_ul': self.checkBox_3.isChecked(),
                'central_buff': self.comboBox_7.currentText(),
                'fortress_buff': self.comboBox_8.currentText(),
                'ring_buff': self.comboBox_9.currentText(),
                'base_buff': self.checkBox_5.isChecked(),
                'dart_target': self.comboBox_6.currentText(),
                'dart_hit_time': self.lineEdit_2.text(),
            }
        }
        return status


class MatchControlPlugin(Plugin):
    """RQt Plugin wrapper for MatchControlWidget"""
    
    def __init__(self, context):
        super(MatchControlPlugin, self).__init__(context)
        self.setObjectName('MatchControlPlugin')
        
        # Create the widget
        self._widget = MatchControlWidget()
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Add widget to the user interface
        context.add_widget(self._widget)
    
    def shutdown_plugin(self):
        """Clean up when plugin is closed"""
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        """Save the plugin settings"""
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the plugin settings"""
        pass


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MatchControlWidget()
    window.setWindowTitle("RM Match Server Sim")
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
