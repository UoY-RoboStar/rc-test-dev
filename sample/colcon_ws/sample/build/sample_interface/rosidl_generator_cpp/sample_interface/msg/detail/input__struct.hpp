// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interface:msg/Input.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_HPP_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_interface__msg__Input __attribute__((deprecated))
#else
# define DEPRECATED__sample_interface__msg__Input __declspec(deprecated)
#endif

namespace sample_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Input_
{
  using Type = Input_<ContainerAllocator>;

  explicit Input_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0ul;
    }
  }

  explicit Input_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0ul;
    }
  }

  // field types and members
  using _value_type =
    uint32_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__value(
    const uint32_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interface::msg::Input_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interface::msg::Input_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interface::msg::Input_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interface::msg::Input_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::Input_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::Input_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interface::msg::Input_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interface::msg::Input_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interface::msg::Input_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interface::msg::Input_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interface__msg__Input
    std::shared_ptr<sample_interface::msg::Input_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interface__msg__Input
    std::shared_ptr<sample_interface::msg::Input_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Input_ & other) const
  {
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const Input_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Input_

// alias to use template instance with default allocator
using Input =
  sample_interface::msg::Input_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interface

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_HPP_
