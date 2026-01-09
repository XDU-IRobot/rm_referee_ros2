#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin

from rm_referee_msgs.msg import DartInfo, DartClientCmd

from rm_referee_mock.dart_client_widget import DartClientWidget
from rm_referee_mock.publisher_pool import PublisherPool


class DartClientPlugin(Plugin):
    """RQt Plugin for dart launch system control"""

    def __init__(self, context):
        super(DartClientPlugin, self).__init__(context)
        self.setObjectName("DartClientPlugin")

        self._widget = DartClientWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()))

        context.add_widget(self._widget)
        self._node = context.node
        self._publisher_pool = PublisherPool(self._node)

        # Create timer for periodic publishing (1Hz for DartInfo, 3Hz for DartClientCmd)
        # We'll publish both at 3Hz to simplify
        self._timer = self._node.create_timer(1.0 / 3.0, self._timer_callback)

    def _timer_callback(self):
        """Timer callback to publish dart messages periodically"""
        topic_prefix = self._widget.get_topic_prefix()

        # Publish DartInfo message (0x0105 - 固定以1Hz频率发送)
        dart_info_data = self._widget.get_dart_info_data()
        dart_info_msg = DartInfo()
        dart_info_msg.header.stamp = self._node.get_clock().now().to_msg()
        dart_info_msg.header.frame_id = "referee_system"
        dart_info_msg.dart_remaining_time = dart_info_data['dart_remaining_time']
        dart_info_msg.dart_info = dart_info_data['dart_info']

        self._publisher_pool.publish(
            f"{topic_prefix}/dart_info", DartInfo, dart_info_msg)

        # Publish DartClientCmd message (0x020A - 固定以3Hz频率发送)
        dart_cmd_data = self._widget.get_dart_client_cmd_data()
        dart_cmd_msg = DartClientCmd()
        dart_cmd_msg.header.stamp = self._node.get_clock().now().to_msg()
        dart_cmd_msg.header.frame_id = "referee_system"
        dart_cmd_msg.dart_launch_opening_status = dart_cmd_data['dart_launch_opening_status']
        dart_cmd_msg.reserved = dart_cmd_data['reserved']
        dart_cmd_msg.target_change_time = dart_cmd_data['target_change_time']
        dart_cmd_msg.latest_launch_cmd_time = dart_cmd_data['latest_launch_cmd_time']

        self._publisher_pool.publish(
            f"{topic_prefix}/dart_client_cmd", DartClientCmd, dart_cmd_msg)

    def shutdown_plugin(self):
        """Clean up when plugin is closed"""
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def save_settings(self, plugin_settings, instance_settings):
        """Save the plugin settings"""
        # TODO: Save widget state if needed
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the plugin settings"""
        # TODO: Restore widget state if needed
        pass
