// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interface:msg/InputAccepted.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__TRAITS_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interface/msg/detail/input_accepted__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace sample_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const InputAccepted & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InputAccepted & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InputAccepted & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sample_interface

namespace rosidl_generator_traits
{

[[deprecated("use sample_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sample_interface::msg::InputAccepted & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interface::msg::InputAccepted & msg)
{
  return sample_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interface::msg::InputAccepted>()
{
  return "sample_interface::msg::InputAccepted";
}

template<>
inline const char * name<sample_interface::msg::InputAccepted>()
{
  return "sample_interface/msg/InputAccepted";
}

template<>
struct has_fixed_size<sample_interface::msg::InputAccepted>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<sample_interface::msg::InputAccepted>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<sample_interface::msg::InputAccepted>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__TRAITS_HPP_
