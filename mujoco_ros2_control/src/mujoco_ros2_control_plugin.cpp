#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

namespace mujoco_ros2_control
{

// ---------------- Static functions -------------------- //
void MujocoRos2ControlPlugin::RegisterPlugin()
{
   mjpPlugin plugin;
   mjp_defaultPlugin(&plugin);

   plugin.name = "mujoco_ros2_control::MujocoRos2ControlPlugin";
   // Allow plugins to be placed on either the body element or the actuator element
   plugin.capabilityflags |= mjPLUGIN_PASSIVE;

   plugin.nattribute = 0;
   plugin.attributes = nullptr;

   plugin.nstate = +[](const mjModel*,  // m
                       int              // plugin_id
                    ) { return 0; };

   plugin.nsensordata = +[](const mjModel*,  // m
                            int,             // plugin_id
                            int              // sensor_id
                         ) { return 0; };

   plugin.needstage = mjSTAGE_VEL;

   plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
      MujocoRos2ControlPlugin* plugin_instance = Create(m, d, plugin_id);
      if (plugin_instance == nullptr)
      {
         return -1;
      }
      if (plugin_instance->initialise(m, d) == false)
         return -1;

      d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
      return 0;
   };

   plugin.destroy = +[](mjData* d, int plugin_id) {
      auto plugin_instance = reinterpret_cast<MujocoRos2ControlPlugin*>(d->plugin_data[plugin_id]);
      plugin_instance->destroy();

      delete plugin_instance;
      d->plugin_data[plugin_id] = 0;
   };

   plugin.reset = +[](const mjModel* m, double*,  // plugin_state
                      void* plugin_data, int plugin_id) {
      auto* plugin_instance = reinterpret_cast<class MujocoRos2ControlPlugin*>(plugin_data);
      plugin_instance->reset(m, plugin_id);
   };

   plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int  // capability_bit
                     ) {
      auto* plugin_instance = reinterpret_cast<class MujocoRos2ControlPlugin*>(d->plugin_data[plugin_id]);
      plugin_instance->compute(m, d, plugin_id);
   };

   mjp_registerPlugin(&plugin);
}

MujocoRos2ControlPlugin* MujocoRos2ControlPlugin::Create(const mjModel* mj_model, mjData* mj_data, int plugin_id)
{

   std::cout << "[MujocoRos2ControlPlugin] Create." << std::endl;

   return new MujocoRos2ControlPlugin();
}

// ------------------------------------------------------ //
// ------------------ Utilis functions ------------------ //

std::string get_robot_description()
{
   // Getting robot description from parameter first. If not set trying from topic
   std::string robot_description;

   auto node = std::make_shared<rclcpp::Node>("robot_description_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

   if (node->has_parameter("robot_description"))
   {
      robot_description = node->get_parameter("robot_description").as_string();
      return robot_description;
   }

   RCLCPP_WARN(node->get_logger(),
               "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
               "topic...");

   auto robot_description_sub = node->create_subscription<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local(), [&](const std_msgs::msg::String::SharedPtr msg) {
      if (!msg->data.empty() && robot_description.empty())
         robot_description = msg->data;
   });

   while (robot_description.empty() && rclcpp::ok())
   {
      rclcpp::spin_some(node);
      RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
   }

   return robot_description;
}

// ------------------------------------------------------ //

bool MujocoRos2ControlPlugin::initialise(const mjModel* mj_model, mjData* mj_data)
{
   int argc = 0;
   char** argv = nullptr;
   if (!rclcpp::ok())
   {
      rclcpp::init(argc, argv);
   }

   // Getting robot description
   std::string urdf_string = get_robot_description();

   // setup actuators and mechanism control node.
   std::vector<hardware_interface::HardwareInfo> control_hardware_info;
   try
   {
      control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
   }
   catch (const std::runtime_error& ex)
   {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_control_plugin"), "Error parsing URDF : " << ex.what());
      return false;
   }

   // Loading hardware interface plugin
   try
   {
      robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterface>("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
   }
   catch (pluginlib::LibraryLoadException& ex)
   {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_control_plugin"), "Failed to create hardware interface loader:  " << ex.what());
      return false;
   }

   // Passing urdf to resource manager
   std::unique_ptr<hardware_interface::ResourceManager> resource_manager = std::make_unique<hardware_interface::ResourceManager>();
   try
   {
      resource_manager->load_urdf(urdf_string, false, false);
   }
   catch (...)
   {
      RCLCPP_ERROR(rclcpp::get_logger("mujoco_ros2_control_plugin"), "Error while initializing URDF!");
   }

   for (const auto& hardware : control_hardware_info)
   {
      std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
      std::unique_ptr<MujocoSystemInterface> mujoco_system;
      try
      {
         mujoco_system = std::unique_ptr<MujocoSystemInterface>(robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
      }
      catch (pluginlib::PluginlibException& ex)
      {
         RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_control_plugin"), "The plugin failed to load. Error: " << ex.what());
         continue;
      }

      urdf::Model urdf_model;
      urdf_model.initString(urdf_string);
      if (!mujoco_system->init_sim(mj_model, mj_data, urdf_model, hardware))
      {
         RCLCPP_FATAL(rclcpp::get_logger("mujoco_ros2_control_plugin"), "Could not initialize robot simulation interface");
         return false;
      }

      resource_manager->import_component(std::move(mujoco_system), hardware);

      rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);
      resource_manager->set_component_state(hardware.name, state);
   }

   // Create the controller manager
   RCLCPP_INFO(rclcpp::get_logger("mujoco_ros2_control_plugin"), "Loading controller_manager");
   cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

   controller_manager_ = std::make_shared<controller_manager::ControllerManager>(std::move(resource_manager), cm_executor_, "controller_manager", "");
   controller_manager_->set_parameter(rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));
   cm_executor_->add_node(controller_manager_);

   auto spin = [this]() { cm_executor_->spin(); };
   cm_thread_ = std::thread(spin);

   if (!controller_manager_->has_parameter("update_rate"))
   {
      RCLCPP_ERROR_STREAM(controller_manager_->get_logger(), "controller manager doesn't have an update_rate parameter");
      return false;
   }

   auto update_rate = controller_manager_->get_update_rate();
   control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

   return true;
}

void MujocoRos2ControlPlugin::compute(const mjModel* mj_model, mjData* mj_data,
                                      int  // plugin_id
)
{
   // Get the simulation time and period
   auto sim_time = mj_data->time;
   int sim_time_sec = static_cast<int>(sim_time);
   int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

   rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
   rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

   if (sim_period >= control_period_)
   {
      controller_manager_->read(sim_time_ros, sim_period);
      controller_manager_->update(sim_time_ros, sim_period);
      last_update_sim_time_ros_ = sim_time_ros;
   }

   // use same time as for read and update call - this is how it is done in ros2_control_node
   controller_manager_->write(sim_time_ros, sim_period);
   return;
}

void MujocoRos2ControlPlugin::destroy()
{
   // rclcpp::shutdown();
   cm_executor_->remove_node(controller_manager_);
   cm_executor_->cancel();

   if (cm_thread_.joinable())
      cm_thread_.join();

   // TODO
   return;
}

void MujocoRos2ControlPlugin::reset(const mjModel*,  // m
                                    int              // plugin_id
)
{
   // free MuJoCo model and data
   // mj_deleteData(mj_data_);

   // TODO
   return;
}

}  // namespace mujoco_ros2_control
