#pragma once

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include "rm_referee_msgs/msg/remote_control.hpp"

namespace rm_referee_mock {

class KeyboardPublisher : public rclcpp::Node {
 public:
  explicit KeyboardPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~KeyboardPublisher();

 private:
  void MainTimerCallback();
  void ProcessKeyboardEvent(const SDL_Event& event, rm_referee_msgs::msg::RemoteControl& msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rm_referee_msgs::msg::RemoteControl>::SharedPtr publisher_;
  SDL_Surface* window_;

  uint16_t keyboard_value_;  ///< 当前的键盘状态

  // ros parameters
  std::string publish_topic_;
  double publish_rate_;
};

}  // namespace rm_referee_mock
