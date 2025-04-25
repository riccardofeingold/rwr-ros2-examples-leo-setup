/**
 * teleop.cpp
 * ROS Node to teleop Franka using real-time joystick commands
 */

#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <array>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <chrono>

#define VEL_SCALER 0.1

class JoyToTwistNode : public rclcpp::Node {
 public:
  JoyToTwistNode()
      : Node("joy_to_twist"), debounce_counter_(0), enabled_(false) {
    // Initialize parameters
    pos_cmd_topic_ = this->declare_parameter<std::string>("pos_cmd_topic", "/right/franka/end_effector_pose_cmd");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/right/franka/end_effector_pose");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");

    // Create publishers
    pos_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pos_cmd_topic_, 10);
    grasp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/hand/right/policy_output", 10);

    // Create subscriber
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10, std::bind(&JoyToTwistNode::poseCallback, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&JoyToTwistNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JoyToTwistNode has been initialized");
  }

 private:
  // Member variables for publishers, subscriber, and parameters.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_cmd_pub_;
  self.saving_done_publisher
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr grasp_pub_;

  std::string pos_cmd_topic_, robot_cmd_topic_, gripper_cmd_topic_, joy_topic_, pose_topic_;
  uint16_t debounce_counter_;
  bool enabled_;
  bool initialized_ = false;
  bool first_time = true;
  std::array<double, 3> position = {0, 0, 0};
  std::array<double, 4> orientation = {1, 0, 0, 0};
  double grasp_width_ = 1.0;
  std::chrono::time_point<std::chrono::steady_clock> current_time_;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (initialized_)
    {
      return;
    }
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;

    orientation[0] = 0.0;
    orientation[1] = 1.0;
    orientation[2] = 0.0;
    orientation[3] = 0.0;
    RCLCPP_INFO(this->get_logger(), "INIT-------------------------------------");
    initialized_ = true;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() != 6) {
      RCLCPP_WARN(this->get_logger(), "Joystick mode mismatch. Switch to mode: \"D\"");
      return;
    }

    std::chrono::duration<double> time_diff = std::chrono::steady_clock::now() - current_time_;
    RCLCPP_INFO_STREAM(this->get_logger(), "COUNT: " << time_diff.count());
    current_time_ =  std::chrono::steady_clock::now(); 
    if(first_time)
    {
      first_time = false;
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

    position[0] -= VEL_SCALER * analogLeft_x * time_diff.count();
    position[1] += VEL_SCALER * analogLeft_y * time_diff.count();
    position[2] += VEL_SCALER * analogRight_y * time_diff.count();

    if (initialized_) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.pose.position.x = position[0];
      pose_msg.pose.position.y = position[1];
      pose_msg.pose.position.z = position[2];

      pose_msg.pose.orientation.w = orientation[0];
      pose_msg.pose.orientation.x = orientation[1];
      pose_msg.pose.orientation.y = orientation[2];
      pose_msg.pose.orientation.z = orientation[3];

      std_msgs::msg::Float32MultiArray grasp_msg;
      grasp_msg.data.resize(1);
      if (buttonR1)
        grasp_width_ += 0.01;
      if (buttonL1)
      {
        grasp_width_ -= 0.01;
      }

      grasp_msg.data[0] = -0.1;
      grasp_pub_->publish(grasp_msg);
      pos_cmd_pub_->publish(pose_msg);
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
