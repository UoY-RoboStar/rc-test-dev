// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interface:msg/Output.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__STRUCT_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sample_interface__msg__Output __attribute__((deprecated))
#else
# define DEPRECATED__sample_interface__msg__Output __declspec(deprecated)
#endif

namespace sample_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Output_
{
  using Type = Output_<ContainerAllocator>;

  explicit Output_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0ul;
    }
  }

  explicit Output_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0ul;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _value_type =
    uint32_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__value(
    const uint32_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interface::msg::Output_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interface::msg::Output_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interface::msg::Output_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interface::msg::Output_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::Output_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::Output_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::Output_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::Output_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interface::msg::Output_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interface::msg::Output_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interface__msg__Output
    std::shared_ptr<sample_interface::msg::Output_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interface__msg__Output
    std::shared_ptr<sample_interface::msg::Output_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Output_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const Output_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Output_

// alias to use template instance with default allocator
using Output =
  sample_interface::msg::Output_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__OUTPUT__STRUCT_HPP_
