/**
Software License Agreement (BSD)

\copyright Copyright (c) 2024, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <memory>
#include <set>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

#include "joy_to_ackermann/joy_to_ackermann.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace joy_to_ackermann
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility.
 */
struct JoyToAckermann::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendAckermannMsg(const sensor_msgs::msg::Joy::SharedPtr joy, bool turbo_mode);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;
  rclcpp::Clock::SharedPtr clock;

  std::string frame_id;
  bool require_enable_button;
  int64_t enable_button;
  int64_t enable_turbo_button;

  int64_t axis_speed;
  int64_t axis_steering;
  
  double scale_speed;
  double scale_speed_turbo;
  double scale_steering;

  bool sent_disable_msg;
};

/**
 * Constructs JoyToAckermann.
 */
JoyToAckermann::JoyToAckermann(const rclcpp::NodeOptions& options) 
  : Node("joy_to_ackermann_node", options)
{
  pimpl_ = new Impl;

  pimpl_->clock = this->get_clock();

  pimpl_->frame_id = this->declare_parameter("frame_id", "base_link");

  pimpl_->ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackerman", 10);

  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS(10),
    std::bind(&JoyToAckermann::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", false);
  pimpl_->enable_button = this->declare_parameter("enable_button", 5);
  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", 4);

  pimpl_->axis_speed = this->declare_parameter("axis_speed", 4);
  pimpl_->axis_steering = this->declare_parameter("axis_steering", 3);

  pimpl_->scale_speed = this->declare_parameter("scale_speed", 1.0);
  pimpl_->scale_speed_turbo = this->declare_parameter("scale_speed_turbo", 2.0);
  pimpl_->scale_steering = this->declare_parameter("scale_steering", 0.5);

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "JoyToAckermann",
      "Enable button %" PRId64 ".", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyToAckermann",
    "Turbo button %" PRId64 ".", pimpl_->enable_turbo_button);

  ROS_INFO_NAMED("JoyToAckermann", "Speed axis %" PRId64 " at scale %f.",
    pimpl_->axis_speed, pimpl_->scale_speed);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyToAckermann",
    "Turbo speed scale %f.", pimpl_->scale_speed_turbo);

  ROS_INFO_NAMED("JoyToAckermann", "Steering axis %" PRId64 " at scale %f.",
    pimpl_->axis_steering, pimpl_->scale_steering);

  pimpl_->sent_disable_msg = false;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_speed", "axis_steering",
                                              "enable_button", "enable_turbo_button"};
    static std::set<std::string> doubleparams = {"scale_speed", "scale_speed_turbo", 
                                                 "scale_steering"};
    static std::set<std::string> boolparams = {"require_enable_button"};
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (boolparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      if (parameter.get_name() == "require_enable_button")
      {
        this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
      }
      else if (parameter.get_name() == "enable_button")
      {
        this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "enable_turbo_button")
      {
        this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_speed")
      {
        this->pimpl_->axis_speed = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_steering")
      {
        this->pimpl_->axis_steering = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "scale_speed")
      {
        this->pimpl_->scale_speed = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_speed_turbo")
      {
        this->pimpl_->scale_speed_turbo = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_steering")
      {
        this->pimpl_->scale_steering = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

JoyToAckermann::~JoyToAckermann()
{
  delete pimpl_;
}

void JoyToAckermann::Impl::sendAckermannMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg,
  bool turbo_mode)
{
  auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
  ackermann_msg->header.stamp = clock->now();
  ackermann_msg->header.frame_id = frame_id;

  // Get speed from joystick axis
  double speed = 0.0;
  if (axis_speed >= 0 && static_cast<int>(joy_msg->axes.size()) > axis_speed)
  {
    double scale = turbo_mode ? scale_speed_turbo : scale_speed;
    speed = joy_msg->axes[axis_speed] * scale;
  }

  // Get steering angle from joystick axis
  double steering = 0.0;
  if (axis_steering >= 0 && static_cast<int>(joy_msg->axes.size()) > axis_steering)
  {
    steering = joy_msg->axes[axis_steering] * scale_steering;
  }

  ackermann_msg->drive.speed = speed;
  ackermann_msg->drive.steering_angle = steering;

  ackermann_pub->publish(std::move(ackermann_msg));
  sent_disable_msg = false;
}

void JoyToAckermann::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (enable_turbo_button >= 0 &&
      static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendAckermannMsg(joy_msg, true);
  }
  else if (!require_enable_button ||
	   (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
           joy_msg->buttons[enable_button]))
  {
    sendAckermannMsg(joy_msg, false);
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
      ackermann_msg->header.stamp = clock->now();
      ackermann_msg->header.frame_id = frame_id;
      // Initializes with zeros by default
      ackermann_pub->publish(std::move(ackermann_msg));
      sent_disable_msg = true;
    }
  }
}

}  // namespace joy_to_ackermann

RCLCPP_COMPONENTS_REGISTER_NODE(joy_to_ackermann::JoyToAckermann)
