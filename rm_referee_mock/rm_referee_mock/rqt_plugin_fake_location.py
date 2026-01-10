#!/usr/bin/env python3

import random
from rqt_gui_py.plugin import Plugin

from rm_referee_msgs.msg import RobotPos, GroundRobotPosition

from rm_referee_mock.fake_location_widget import FakeLocationWidget
from rm_referee_mock.publisher_pool import PublisherPool


class FakeLocationPlugin(Plugin):
    """RQt Plugin for fake location data publishing"""
    
    def __init__(self, context):
        super(FakeLocationPlugin, self).__init__(context)
        self.setObjectName("FakeLocationPlugin")
        
        self._widget = FakeLocationWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        
        context.add_widget(self._widget)
        self._node = context.node
        self._publisher_pool = PublisherPool(self._node)
        
        # Initialize timer
        self._timer = None
        self._current_rate = 0.0
        
        # Create initial timer
        self._update_timer()
        
        # Connect to widget's rate change signal to update timer
        self._widget.rate_edit.editingFinished.connect(self._update_timer)
    
    def _add_noise(self, value):
        """Add Gaussian noise to a value based on widget's noise scale setting"""
        noise_scale = self._widget.get_noise_scale()
        if noise_scale > 0:
            return value + random.gauss(0, noise_scale)
        return value
    
    def _update_timer(self):
        """Update timer when rate changes"""
        new_rate = self._widget.get_publish_rate()
        
        # Check if we need to update
        rate_changed = abs(new_rate - self._current_rate) > 0.001
        
        if not rate_changed:
            return
        
        # Cancel old timer if exists
        if self._timer:
            self._timer.cancel()
            self._timer = None
        
        # Create new timer with updated rate
        if new_rate > 0:
            self._timer = self._node.create_timer(1.0 / new_rate, self._timer_callback)
            self._current_rate = new_rate
    
    def _timer_callback(self):
        """Timer callback to publish location messages periodically"""
        topic_prefix = self._widget.get_topic_prefix()
        
        # Publish RobotPos message
        robot_pos_data = self._widget.get_robot_pos_data()
        robot_pos_msg = RobotPos()
        robot_pos_msg.header.stamp = self._node.get_clock().now().to_msg()
        robot_pos_msg.header.frame_id = "referee_system"
        robot_pos_msg.x = self._add_noise(robot_pos_data['x'])
        robot_pos_msg.y = self._add_noise(robot_pos_data['y'])
        robot_pos_msg.angle = robot_pos_data['angle']
        
        self._publisher_pool.publish(
            f"{topic_prefix}/robot_pos", RobotPos, robot_pos_msg)
        
        # Publish GroundRobotPosition message
        ground_pos_data = self._widget.get_ground_robot_position_data()
        ground_pos_msg = GroundRobotPosition()
        ground_pos_msg.header.stamp = self._node.get_clock().now().to_msg()
        ground_pos_msg.header.frame_id = "referee_system"
        ground_pos_msg.hero_x = self._add_noise(ground_pos_data['hero_x'])
        ground_pos_msg.hero_y = self._add_noise(ground_pos_data['hero_y'])
        ground_pos_msg.engineer_x = self._add_noise(ground_pos_data['engineer_x'])
        ground_pos_msg.engineer_y = self._add_noise(ground_pos_data['engineer_y'])
        ground_pos_msg.standard_3_x = self._add_noise(ground_pos_data['standard_3_x'])
        ground_pos_msg.standard_3_y = self._add_noise(ground_pos_data['standard_3_y'])
        ground_pos_msg.standard_4_x = self._add_noise(ground_pos_data['standard_4_x'])
        ground_pos_msg.standard_4_y = self._add_noise(ground_pos_data['standard_4_y'])
        ground_pos_msg.reserved = ground_pos_data['reserved']
        ground_pos_msg.reserved_2 = ground_pos_data['reserved_2']
        
        self._publisher_pool.publish(
            f"{topic_prefix}/ground_robot_position", GroundRobotPosition, ground_pos_msg)
    
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
