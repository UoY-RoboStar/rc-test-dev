// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interface:msg/InputAccepted.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_HPP_

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
# define DEPRECATED__sample_interface__msg__InputAccepted __attribute__((deprecated))
#else
# define DEPRECATED__sample_interface__msg__InputAccepted __declspec(deprecated)
#endif

namespace sample_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InputAccepted_
{
  using Type = InputAccepted_<ContainerAllocator>;

  explicit InputAccepted_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    (void)_init;
  }

  explicit InputAccepted_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interface::msg::InputAccepted_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interface::msg::InputAccepted_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::InputAccepted_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::InputAccepted_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interface__msg__InputAccepted
    std::shared_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interface__msg__InputAccepted
    std::shared_ptr<sample_interface::msg::InputAccepted_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InputAccepted_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const InputAccepted_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InputAccepted_

// alias to use template instance with default allocator
using InputAccepted =
  sample_interface::msg::InputAccepted_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_HPP_
