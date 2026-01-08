
#include "rm_referee_mock/keyboard_publisher.h"

namespace rm_referee_mock {

KeyboardPublisher::KeyboardPublisher(const rclcpp::NodeOptions& options)
    : Node("keyboard_publisher", options), keyboard_value_(0), window_(nullptr) {
  // Declare and get parameters
  this->declare_parameter("publish_topic", "/rm_referee/mock/remote_control");
  this->declare_parameter("publish_rate", 20.0);

  publish_topic_ = this->get_parameter("publish_topic").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  // Create publisher
  publisher_ = this->create_publisher<rm_referee_msgs::msg::RemoteControl>(publish_topic_, 10);

  // Initialize SDL
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not init SDL: %s", SDL_GetError());
    throw std::runtime_error("Could not init SDL");
  }

  SDL_EnableKeyRepeat(0, 0);  // Disable key repeat
  SDL_WM_SetCaption("RM Referee Mock - Keyboard Publisher", NULL);
  window_ = SDL_SetVideoMode(300, 100, 0, 0);

  if (!window_) {
    RCLCPP_ERROR(this->get_logger(), "Could not create SDL window: %s", SDL_GetError());
    SDL_Quit();
    throw std::runtime_error("Could not create SDL window");
  }

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                   std::bind(&KeyboardPublisher::MainTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Keyboard publisher initialized.");
  RCLCPP_INFO(this->get_logger(), "Topic: %s", publish_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Rate: %.1f Hz", publish_rate_);
  RCLCPP_INFO(this->get_logger(), "Please focus on the SDL window and use keyboard for input");
}

KeyboardPublisher::~KeyboardPublisher() {
  if (window_) {
    SDL_FreeSurface(window_);
  }
  SDL_Quit();
}

void KeyboardPublisher::ProcessKeyboardEvent(const SDL_Event& event, rm_referee_msgs::msg::RemoteControl& msg) {
  if (event.type == SDL_KEYDOWN) {
    // Visual feedback - green when key pressed
    SDL_FillRect(window_, NULL, SDL_MapRGB(window_->format, 0, 255, 0));
    // Map SDL key codes to RemoteControl keyboard bits
    switch (event.key.keysym.sym) {
      case SDLK_w:
        keyboard_value_ |= msg.KEY_W;
        break;
      case SDLK_s:
        keyboard_value_ |= msg.KEY_S;
        break;
      case SDLK_a:
        keyboard_value_ |= msg.KEY_A;
        break;
      case SDLK_d:
        keyboard_value_ |= msg.KEY_D;
        break;
      case SDLK_LSHIFT:
        keyboard_value_ |= msg.KEY_LSHIFT;
        break;
      case SDLK_LCTRL:
        keyboard_value_ |= msg.KEY_LCTRL;
        break;
      case SDLK_q:
        keyboard_value_ |= msg.KEY_Q;
        break;
      case SDLK_e:
        keyboard_value_ |= msg.KEY_E;
        break;
      case SDLK_r:
        keyboard_value_ |= msg.KEY_R;
        break;
      case SDLK_f:
        keyboard_value_ |= msg.KEY_F;
        break;
      case SDLK_g:
        keyboard_value_ |= msg.KEY_G;
        break;
      case SDLK_z:
        keyboard_value_ |= msg.KEY_Z;
        break;
      case SDLK_x:
        keyboard_value_ |= msg.KEY_X;
        break;
      case SDLK_c:
        keyboard_value_ |= msg.KEY_C;
        break;
      case SDLK_v:
        keyboard_value_ |= msg.KEY_V;
        break;
      case SDLK_b:
        keyboard_value_ |= msg.KEY_B;
        break;
      default:
        // 如果按的是裁判系统协议里面没有的按键，窗口显示红色，代表输入无效
        SDL_FillRect(window_, NULL, SDL_MapRGB(window_->format, 255, 0, 0));
        break;
    }

    SDL_Flip(window_);

  } else if (event.type == SDL_KEYUP) {
    // Clear the corresponding bit
    switch (event.key.keysym.sym) {
      case SDLK_w:
        keyboard_value_ &= ~msg.KEY_W;
        break;
      case SDLK_s:
        keyboard_value_ &= ~msg.KEY_S;
        break;
      case SDLK_a:
        keyboard_value_ &= ~msg.KEY_A;
        break;
      case SDLK_d:
        keyboard_value_ &= ~msg.KEY_D;
        break;
      case SDLK_LSHIFT:
        keyboard_value_ &= ~msg.KEY_LSHIFT;
        break;
      case SDLK_LCTRL:
        keyboard_value_ &= ~msg.KEY_LCTRL;
        break;
      case SDLK_q:
        keyboard_value_ &= ~msg.KEY_Q;
        break;
      case SDLK_e:
        keyboard_value_ &= ~msg.KEY_E;
        break;
      case SDLK_r:
        keyboard_value_ &= ~msg.KEY_R;
        break;
      case SDLK_f:
        keyboard_value_ &= ~msg.KEY_F;
        break;
      case SDLK_g:
        keyboard_value_ &= ~msg.KEY_G;
        break;
      case SDLK_z:
        keyboard_value_ &= ~msg.KEY_Z;
        break;
      case SDLK_x:
        keyboard_value_ &= ~msg.KEY_X;
        break;
      case SDLK_c:
        keyboard_value_ &= ~msg.KEY_C;
        break;
      case SDLK_v:
        keyboard_value_ &= ~msg.KEY_V;
        break;
      case SDLK_b:
        keyboard_value_ &= ~msg.KEY_B;
        break;
    }

    // Visual feedback - grey when key released
    SDL_FillRect(window_, NULL, SDL_MapRGB(window_->format, 128, 128, 128));
    SDL_Flip(window_);
  }
}

void KeyboardPublisher::MainTimerCallback() {
  // Process all pending SDL events
  SDL_Event event;
  auto msg = rm_referee_msgs::msg::RemoteControl();

  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT) {
      RCLCPP_INFO(this->get_logger(), "SDL window closed, shutting down...");
      rclcpp::shutdown();
      return;
    }

    ProcessKeyboardEvent(event, msg);
  }

  // Publish current keyboard state
  msg.header.stamp = this->get_clock()->now();
  msg.keyboard_value = keyboard_value_;
  msg.mouse_x = 0;
  msg.mouse_y = 0;
  msg.mouse_z = 0;
  msg.left_button_down = 0;
  msg.right_button_down = 0;
  msg.reserved = 0;

  publisher_->publish(msg);
}

}  // namespace rm_referee_mock

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_referee_mock::KeyboardPublisher)
