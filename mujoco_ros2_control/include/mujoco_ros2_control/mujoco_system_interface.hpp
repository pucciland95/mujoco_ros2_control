#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco/mujoco.h"

namespace mujoco_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  virtual bool init_sim(rclcpp::Node::SharedPtr & node, mjModel* mujoco_model, mjData *mujoco_data, // shared ptr
    const hardware_interface::HardwareInfo & hardware_info) = 0;

protected:
  rclcpp::Node::SharedPtr node_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_