// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interface:msg/InputAccepted.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__BUILDER_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interface/msg/detail/input_accepted__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interface
{

namespace msg
{

namespace builder
{

class Init_InputAccepted_stamp
{
public:
  Init_InputAccepted_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sample_interface::msg::InputAccepted stamp(::sample_interface::msg::InputAccepted::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interface::msg::InputAccepted msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interface::msg::InputAccepted>()
{
  return sample_interface::msg::builder::Init_InputAccepted_stamp();
}

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__BUILDER_HPP_
