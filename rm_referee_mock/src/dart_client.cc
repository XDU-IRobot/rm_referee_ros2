
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <rclcpp/rclcpp.hpp>
#include "rm_referee_msgs/msg/dart_client_cmd.hpp"

namespace rm_referee_mock {

class DartClient : public rclcpp::Node {
 public:
  explicit DartClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("dart_client", options) {
    // Declare and get parameters
    this->declare_parameter("publish_topic", "/rm_referee/dart_client_cmd");
    this->declare_parameter("publish_rate", 10.0);

    publish_topic_ = this->get_parameter("publish_topic").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    // Create publisher
    publisher_ = this->create_publisher<rm_referee_msgs::msg::DartClientCmd>(publish_topic_, 10);

    // Initialize terminal for non-blocking keyboard input
    if (!SetupTerminal()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup terminal");
      throw std::runtime_error("Failed to setup terminal");
    }

    // Create timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                     std::bind(&DartClient::MainTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Topic: %s", publish_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Reading keyboard input from terminal:");
    RCLCPP_INFO(this->get_logger(), "  Press 1 to set dart_launch_opening_status to 0");
    RCLCPP_INFO(this->get_logger(), "  Press 2 to set dart_launch_opening_status to 1");
    RCLCPP_INFO(this->get_logger(), "  Press 3 to set dart_launch_opening_status to 2");
    RCLCPP_INFO(this->get_logger(), "  Press 'l' to update latest_launch_cmd_time to current timestamp");
  }

  ~DartClient() { RestoreTerminal(); }

 private:
  bool SetupTerminal() {
    // Get current terminal settings
    if (tcgetattr(STDIN_FILENO, &original_term_) < 0) {
      return false;
    }

    // Set terminal to non-canonical mode (no line buffering)
    struct termios raw = original_term_;
    raw.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    raw.c_cc[VMIN] = 0;               // Non-blocking read
    raw.c_cc[VTIME] = 0;              // No timeout

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
      return false;
    }

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    return true;
  }

  void RestoreTerminal() {
    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
  }

  void MainTimerCallback() {
    // Read keyboard input
    char c;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      ProcessKeyboardInput(c);
    }

    // Publish current state
    dart_client_cmd_msg_.header.stamp = this->get_clock()->now();
    publisher_->publish(dart_client_cmd_msg_);
  }

  void ProcessKeyboardInput(char c) {
    switch (c) {
      case '1':
        dart_client_cmd_msg_.dart_launch_opening_status = 0;
        RCLCPP_INFO(this->get_logger(), "dart_launch_opening_status set to 0");
        break;
      case '2':
        dart_client_cmd_msg_.dart_launch_opening_status = 1;
        RCLCPP_INFO(this->get_logger(), "dart_launch_opening_status set to 1");
        break;
      case '3':
        dart_client_cmd_msg_.dart_launch_opening_status = 2;
        RCLCPP_INFO(this->get_logger(), "dart_launch_opening_status set to 2");
        break;
      case 'l':
      case 'L':
        // Get current time in milliseconds since epoch
        dart_client_cmd_msg_.latest_launch_cmd_time = static_cast<uint16_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count() &
            0xFFFF);  // Keep only lower 16 bits
        RCLCPP_INFO(this->get_logger(), "latest_launch_cmd_time updated to %u",
                    dart_client_cmd_msg_.latest_launch_cmd_time);
        break;
      case 'q':
      case 'Q':
      case '\x03':  // Ctrl-C
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
        break;
      default:
        // Ignore unknown keys
        break;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rm_referee_msgs::msg::DartClientCmd>::SharedPtr publisher_;

  struct termios original_term_;

  rm_referee_msgs::msg::DartClientCmd dart_client_cmd_msg_{};

  // ROS parameters
  std::string publish_topic_;
  double publish_rate_;
};

}  // namespace rm_referee_mock

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_referee_mock::DartClient)
