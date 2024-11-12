// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interface:msg/Input.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT__TRAITS_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interface/msg/detail/input__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Input & msg,
  std::ostream & out)
{
  out << "{";
  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Input & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Input & msg, bool use_flow_style = false)
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
  const sample_interface::msg::Input & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interface::msg::Input & msg)
{
  return sample_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interface::msg::Input>()
{
  return "sample_interface::msg::Input";
}

template<>
inline const char * name<sample_interface::msg::Input>()
{
  return "sample_interface/msg/Input";
}

template<>
struct has_fixed_size<sample_interface::msg::Input>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_interface::msg::Input>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_interface::msg::Input>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT__TRAITS_HPP_
