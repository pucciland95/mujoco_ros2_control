#ifndef MUJOCO_ROS2_CONTROL_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGIN_HPP_

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "pluginlib/class_loader.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

// Mujoco
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <limits>
#include <string>
#include <cstdlib>

namespace mujoco_ros2_control
{

class MJResourceManager : public hardware_interface::ResourceManager
{
 public:
   MJResourceManager(rclcpp::Node::SharedPtr& node, const mjModel* mj_model, mjData* mj_data)
     : hardware_interface::ResourceManager(node->get_node_clock_interface(), node->get_node_logging_interface())
     , mj_system_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface")
     , logger_(node->get_logger().get_child("MJResourceManager"))
   {
      node_ = node;
      mj_model_ = mj_model;
      mj_data_ = mj_data;
   }

   MJResourceManager(const MJResourceManager&) = delete;

   ~MJResourceManager() override
   {
      RCLCPP_WARN(logger_, "Called destructor of MJResourceManager!");
      // Should this do something else?
   }

   // Called from Controller Manager when robot description is initialized from callback
   bool load_and_initialize_components(const std::string& urdf, unsigned int update_rate) override
   {
      components_are_loaded_and_initialized_ = true;

      const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

      urdf::Model urdf_model;
      urdf_model.initString(urdf);

      for (const auto& individual_hardware_info : hardware_info)
      {
         std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;

         // Load hardware
         std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;
         std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
         try
         {
            mujoco_system = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(mj_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
         }
         catch (pluginlib::PluginlibException& ex)
         {
            RCLCPP_ERROR(logger_, "The plugin failed to load for some reason. Error: %s\n", ex.what());
            continue;
         }

         // initialize simulation requirements
         if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, individual_hardware_info))
         {
            RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
            components_are_loaded_and_initialized_ = false;
            break;
         }
         RCLCPP_DEBUG(logger_, "Initialized robot simulation interface %s!", robot_hw_sim_type_str_.c_str());

         // initialize hardware
         import_component(std::move(mujoco_system), individual_hardware_info);
      }

      return components_are_loaded_and_initialized_;
   }

 private:
   std::shared_ptr<rclcpp::Node> node_;
   const mjModel* mj_model_;
   mjData* mj_data_;

   /// \brief Interface loader
   pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> mj_system_loader_;
   rclcpp::Logger logger_;
};

/** \brief Plugin to let ros2_controllers control the robot in Mujoco via ROS topic. */
class MujocoRos2ControlPlugin
{
 public:
   /** \brief Register plugin. */
   static void RegisterPlugin();

   /** \brief Create an instance of the plugin.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
   static MujocoRos2ControlPlugin* Create(const mjModel* m, mjData* d, int plugin_id);

 public:
   /** \brief Copy constructor. */
   MujocoRos2ControlPlugin(MujocoRos2ControlPlugin&&) = default;

   /** \brief Constructor.
     \param m model
     \param d data
     \param actuator_id actuator ID
     \param topic_name topic name
   */
   MujocoRos2ControlPlugin(std::string controller_to_load_name) : time_since_sim_started(0, 0, RCL_ROS_TIME), last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
   {
      controllers_to_load_name_.push_back(controller_to_load_name);
   }

   ~MujocoRos2ControlPlugin()
   {
      RCLCPP_INFO(controller_manager_->get_logger(), "Called MujocoRos2ControlPlugin destructor");
   }

   /** \brief Initialises plugin.
    \param mj_model mujoco model
    \param mj_data mujoco data
    */
   bool initialise(const mjModel* mj_model, mjData* mj_data);

   /** \brief Reset.
    \param m model
    \param plugin_id plugin ID
    */
   void reset(const mjModel* m, int plugin_id);

   /** \brief Destroy.
    \param m model
    \param plugin_id plugin ID
    */
   void destroy();

   /** \brief Compute.
    \param m model
    \param d data
    \param plugin_id plugin ID
    */
   void compute(const mjModel* m, mjData* d, int plugin_id);

   /** \brief Initialise controller manager and all its resources.
    \param m model
    \param d data
    */
   bool initialise_controller_manager(const mjModel* mj_model, mjData* mj_data);

   /** \brief checks whether the controllers have been initialised.
    */
   bool are_controllers_initialised();

   /** \brief loads the controllers have been passed to the mujoco plugin (aka: controllers_to_load_name_).
    */
   bool load_controllers();

   /** \brief unloads the controllers have been passed to the mujoco plugin (aka: controllers_to_load_name_).
    */
   bool unload_controllers();

   /** \brief executes through CLI the spawn_controllers.launch.py file that loads the ros controllers. Notice that it is blocking
    * and therefore should be launched on a parallel threah
    */
   bool launch_controllers();


 protected:
   // ROS variablesd
   rclcpp::Node::SharedPtr node_;
   rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
   rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);
   rclcpp::Time time_since_sim_started;
   rclcpp::Time last_update_sim_time_ros_;
   std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
   // std::string robot_description_;
   // mujoco_ros2_control::MujocoSystemInterface *p_mujoco_system_;
   std::vector<std::string> controllers_to_load_name_ = { "joint_state_broadcaster" };

   // Non ROS variables
   std::thread cm_thread_;
   mjData* mj_data_;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL_PLUGIN_HPP_
