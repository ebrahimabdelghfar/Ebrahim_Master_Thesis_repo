#ifndef JOY_TO_ACKERMANN__JOY_TO_ACKERMANN_HPP_
#define JOY_TO_ACKERMANN__JOY_TO_ACKERMANN_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joy_to_ackermann/joy_to_ackermann_export.h"

namespace joy_to_ackermann
{

class JOY_TO_ACKERMANN_EXPORT JoyToAckermann : public rclcpp::Node
{
public:
  explicit JoyToAckermann(const rclcpp::NodeOptions & options);
  virtual ~JoyToAckermann();

private:
  struct Impl;
  Impl * pimpl_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;
};

}  // namespace joy_to_ackermann

#endif  // JOY_TO_ACKERMANN__JOY_TO_ACKERMANN_HPP_
