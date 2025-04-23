#include <mujoco/mjplugin.h>
#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

namespace mujoco_ros2_control
{

mjPLUGIN_LIB_INIT { MujocoRos2ControlPlugin::RegisterPlugin(); }

}  // namespace mujoco_ros2_control
