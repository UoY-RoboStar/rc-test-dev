// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interface:msg/Input.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT__BUILDER_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interface/msg/detail/input__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interface
{

namespace msg
{

namespace builder
{

class Init_Input_value
{
public:
  Init_Input_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sample_interface::msg::Input value(::sample_interface::msg::Input::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interface::msg::Input msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interface::msg::Input>()
{
  return sample_interface::msg::builder::Init_Input_value();
}

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT__BUILDER_HPP_
