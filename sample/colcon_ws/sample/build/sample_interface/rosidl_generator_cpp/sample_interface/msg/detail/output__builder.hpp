// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interface:msg/Output.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__BUILDER_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interface/msg/detail/output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interface
{

namespace msg
{

namespace builder
{

class Init_Output_value
{
public:
  explicit Init_Output_value(::sample_interface::msg::Output & msg)
  : msg_(msg)
  {}
  ::sample_interface::msg::Output value(::sample_interface::msg::Output::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interface::msg::Output msg_;
};

class Init_Output_stamp
{
public:
  Init_Output_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Output_value stamp(::sample_interface::msg::Output::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Output_value(msg_);
  }

private:
  ::sample_interface::msg::Output msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interface::msg::Output>()
{
  return sample_interface::msg::builder::Init_Output_stamp();
}

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__BUILDER_HPP_
