// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

// #include "control_toolbox/pid.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace mujoco_ros2_control
{

class MujocoSystem : public MujocoSystemInterface
{
public:
  MujocoSystem();
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

  bool init_sim(
    const mjModel *mujoco_model, mjData *mujoco_data, const urdf::Model &urdf_model,
    const hardware_interface::HardwareInfo &hardware_info) override;

  bool reset_sim() override {set_initial_pose(); return true;}

  struct JointState
  {
    std::string name;
    double initial_position;
    double position;
    double initial_velocity;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
    double min_position_command;
    double max_position_command;
    double min_velocity_command;
    double max_velocity_command;
    double min_effort_command;
    double max_effort_command;
    bool is_position_control_enabled{false};
    bool is_velocity_control_enabled{false};
    bool is_effort_control_enabled{false};
    joint_limits::JointLimits joint_limits;
    int mj_joint_type;
    int mj_pos_adr;
    int mj_vel_adr;
  };

private:
  void register_joints(
    const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info);
  void set_initial_pose();
  void get_joint_limits(
    urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits &joint_limits);
  double clamp(double v, double lo, double hi) { return (v < lo) ? lo : (hi < v) ? hi : v; }

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<JointState> joint_states_;

  const mjModel *mj_model_;
  mjData *mj_data_;

  rclcpp::Logger logger_;
  bool initial_pose_set_ = false; // TODO: remove
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
