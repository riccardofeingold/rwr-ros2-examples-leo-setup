/**
 * teleop.cpp
 * ROS Node to teleop Franka using real-time joystick commands
 */

#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyToTwistNode : public rclcpp::Node {
 public:
  JoyToTwistNode()
      : Node("joy_to_twist"), debounce_counter_(0), enabled_(false) {
    // Initialize parameters
    twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/franka_twist");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");

    // Create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);

    // Create subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&JoyToTwistNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JoyToTwistNode has been initialized");
  }

 private:
  // Member variables for publishers, subscriber, and parameters.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::string twist_topic_, robot_cmd_topic_, gripper_cmd_topic_, joy_topic_;
  uint16_t debounce_counter_;
  bool enabled_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() != 6) {
      RCLCPP_WARN(this->get_logger(), "Joystick mode mismatch. Switch to mode: \"D\"");
      return;
    }

    // axes:
    const double &analogLeft_x{msg->axes[0]};
    const double &analogLeft_y{msg->axes[1]};
    const double &analogRight_x{msg->axes[2]};
    const double &analogRight_y{msg->axes[3]};
    const double &dPad_x{msg->axes[4]};
    const double &dPad_y{msg->axes[5]};

    // buttons:
    const int &buttonX{msg->buttons[0]};
    const int &buttonA{msg->buttons[1]};
    const int &buttonB{msg->buttons[2]};
    [[maybe_unused]] const int &buttonY{msg->buttons[3]};
    [[maybe_unused]] const int &buttonL1{msg->buttons[4]};
    [[maybe_unused]] const int &buttonR1{msg->buttons[5]};
    [[maybe_unused]] const int &buttonL2{msg->buttons[6]};
    [[maybe_unused]] const int &buttonR2{msg->buttons[7]};

    const bool &buttonBack{static_cast<bool>(msg->buttons[8])};
    const bool &buttonStart{static_cast<bool>(msg->buttons[9])};

    const double scaling{1.0};

    // Handle joystick enable/disable logic with debouncing
    if (buttonStart) {
      debounce_counter_++;
    } else if (buttonBack) {
      debounce_counter_ = 0;
    }
    bool enabled_prev = enabled_;
    enabled_ = debounce_counter_ > 20;
    if (enabled_prev != enabled_) {
      RCLCPP_INFO(this->get_logger(), "{%s}: Joystick to Twist", (enabled_ ? "ENABLED" : "DISABLED"));
    }

    // Publish twist commands if enabled
    if (enabled_) {
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = scaling * analogRight_y;
      twist_msg.linear.y = scaling * analogRight_x;
      twist_msg.linear.z = scaling * dPad_y;
      twist_msg.angular.x = -scaling * analogLeft_x;
      twist_msg.angular.y = scaling * analogLeft_y;
      twist_msg.angular.z = scaling * dPad_x;
      twist_pub_->publish(twist_msg);
    }

  }
};

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create the JoyToTwistNode and spin it
  auto node = std::make_shared<JoyToTwistNode>();
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
