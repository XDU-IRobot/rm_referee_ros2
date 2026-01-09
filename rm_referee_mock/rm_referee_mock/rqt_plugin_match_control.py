#!/usr/bin/env python3

from time import time

from rqt_gui_py.plugin import Plugin

from rm_referee_msgs.msg import GameStatus, GameRobotHP

from rm_referee_mock.match_control_widget import MatchControlWidget
from rm_referee_mock.publisher_pool import PublisherPool


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
        self._timer = self._node.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        """Timer callback to publish match status periodically"""
        # Get current status from widget
        status = self._widget.get_game_status()
        topic_prefix = self._widget.get_topic_prefix()

        # Publish GameStatus message
        game_status_msg = GameStatus()
        game_status_msg.header.stamp = self._node.get_clock().now().to_msg()
        game_status_msg.header.frame_id = "referee_system"
        game_status_msg.game_type = int(status["game_type"])
        game_status_msg.game_progress = int(status["game_progress"])
        game_status_msg.stage_remain_time = int(status["stage_remain_time"])
        game_status_msg.sync_timestamp = int(time() * 1000)

        self._publisher_pool.publish(
            f"{topic_prefix}/game_status", GameStatus, game_status_msg)

        # Publish GameRobotHP message
        robot_hp_msg = GameRobotHP()
        robot_hp_msg.header.stamp = self._node.get_clock().now().to_msg()
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
