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

   const char* attributes[] = { "controller_to_load_name" };

   plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
   plugin.attributes = attributes;

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
   const char* controller_to_load_name = mj_getPluginConfig(mj_model, plugin_id, "controller_to_load_name");
   if (strlen(controller_to_load_name) == 0)
   {
      mju_error("[mujoco_ros2_control] `controller_to_load_name` is missing.");
      return nullptr;
   }
   std::string controller_to_load_name_str = std::string(controller_to_load_name);

   return new MujocoRos2ControlPlugin(controller_to_load_name_str);
}

// ------------------------------------------------------ //
// ------------- Mujoco plugin functions ---------------- //

bool MujocoRos2ControlPlugin::initialise(const mjModel* mj_model, mjData* mj_data)
{
   this->node_ = rclcpp::Node::make_shared("mujoco_ros2_control_plugin");
   this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
   this->executor_->add_node(this->node_);

   auto spin = [this]() { executor_->spin(); };
   cm_thread_ = std::thread(spin);

   mj_data_ = mj_data;

   bool ok = initialise_controller_manager(mj_model, mj_data_);

   return ok;
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
      time_since_sim_started += sim_period;
      controller_manager_->read(time_since_sim_started, sim_period);
      controller_manager_->update(time_since_sim_started, sim_period);
      last_update_sim_time_ros_ = sim_time_ros;
   }

   // use same time as for read and update call - this is how it is done in ros2_control_node
   controller_manager_->write(time_since_sim_started, sim_period);
   return;
}

void MujocoRos2ControlPlugin::destroy()
{
   executor_->remove_node(controller_manager_);
   executor_->remove_node(node_);
   executor_->cancel();

   if (cm_thread_.joinable())
      cm_thread_.join();

   // TODO
   return;
}


void MujocoRos2ControlPlugin::reset(const mjModel* mj_model,
                                    int  // plugin_id
)
{
   this->unload_controllers();
   this->load_controllers();

   last_update_sim_time_ros_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

   rclcpp::Duration sim_period = rclcpp::Duration(1, 0);           // Is it useful?
   controller_manager_->read(time_since_sim_started, sim_period);  // Is it useful?

   // TODO: add correct time (i.e. time required for the whole reset funcition)
   time_since_sim_started += sim_period;  // Is it useful?

   return;
}

// ------------------------------------------------------ //
// --------------- ros_control functions ---------------- //

bool MujocoRos2ControlPlugin::initialise_controller_manager(const mjModel* mj_model, mjData* mj_data)
{
   // Create the resource manager
   std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ = std::make_unique<mujoco_ros2_control::MJResourceManager>(this->node_, mj_model, mj_data);

   // Create the controller manager
   std::string controller_manager_node_name = "controller_manager";
   rclcpp::NodeOptions options = controller_manager::get_cm_node_options();

   this->controller_manager_ =
       std::make_shared<controller_manager::ControllerManager>(std::move(resource_manager_), this->executor_, controller_manager_node_name, this->node_->get_namespace(), options);
   this->executor_->add_node(this->controller_manager_);

   if (!controller_manager_->has_parameter("update_rate"))
   {
      RCLCPP_ERROR_STREAM(controller_manager_->get_logger(), "controller manager doesn't have an update_rate parameter");
      return false;
   }

   auto update_rate = controller_manager_->get_update_rate();
   control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

   // Wait for CM to receive robot description from the topic and then initialize Resource Manager
   while (!controller_manager_->is_resource_manager_initialized())
   {
      RCLCPP_WARN(node_->get_logger(), "Waiting RM to load and initialize hardware...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
   }

   // Waiting 4 controllers to be initialised in a different thread
   bool init_thread_finished = false;
   auto init_controllers = [this, &init_thread_finished]() {
      while (are_controllers_initialised() == false)
      {
         using namespace std::chrono_literals;
         std::this_thread::sleep_for(1s);
      }
      init_thread_finished = true;
   };
   std::thread init_controller_thread = std::thread(init_controllers);

   // Launching the controllers on a different thread
   bool launch_thread_ok = true;
   auto launch_controllers_lamda = [this, &launch_thread_ok]() {
      launch_thread_ok = launch_controllers();
   };
   std::thread launch_controller_thread = std::thread(launch_controllers_lamda);

   // Spinning controller manager untill init_thread_finished is finished
   while (init_thread_finished != true)
   {
      rclcpp::Duration sim_period = rclcpp::Duration(1, 0);
      controller_manager_->update(time_since_sim_started, sim_period);

      if(launch_thread_ok == false)
         return false;

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
   }

   if (launch_controller_thread.joinable())
      launch_controller_thread.join();

   if (init_controller_thread.joinable())
      init_controller_thread.join();

   RCLCPP_WARN(controller_manager_->get_logger(), "Finished to initialise mujoco controller manager");

   return true;
}

bool MujocoRos2ControlPlugin::are_controllers_initialised()
{
   // Check that controllers have been loaded
   std::vector<controller_manager::ControllerSpec> controllers_specs = controller_manager_->get_loaded_controllers();

   if (controllers_specs.size() <= 0)
   {
      RCLCPP_WARN(node_->get_logger(), "No controller loaded yet...");
      return false;
   }

   for (std::string controller_name : this->controllers_to_load_name_)
   {
      auto controller_has_name = [controller_name](const controller_manager::ControllerSpec& other_controller) -> bool { return other_controller.info.name == controller_name; };

      // Check that controller has been loaded
      auto it = std::find_if(controllers_specs.begin(), controllers_specs.end(), controller_has_name);
      if (it == controllers_specs.end())
      {
         RCLCPP_WARN(node_->get_logger(), "%s controller has not been yet loaded!", controller_name.c_str());
         return false;
      }

      // Check that ros controller is ACTIVE
      if (it->c->get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
         RCLCPP_WARN(node_->get_logger(), "%s controller has not been yet been initialised!", controller_name.c_str());
         return false;
      }
   }

   return true;
}

bool MujocoRos2ControlPlugin::load_controllers()
{
   bool load_thread_finished = false;
   auto load_controllers = [this, &load_thread_finished]() {
      std::vector<std::string> no_controllers = {};
      if (controllers_to_load_name_.empty() == false)
      {
         rclcpp::Duration timeout = rclcpp::Duration(5, 0);
         if (controller_manager_->switch_controller(controllers_to_load_name_, no_controllers, 2, false, timeout) == controller_interface::return_type::ERROR)
         {
            RCLCPP_ERROR(controller_manager_->get_logger(), "Failed to deactivate controllers");
            return;
         }
      }

      load_thread_finished = true;
   };
   std::thread load_controller_thread = std::thread(load_controllers);

   rclcpp::Duration sim_period = rclcpp::Duration(1, 0);
   while (load_thread_finished != true)
   {
      controller_manager_->update(time_since_sim_started, sim_period);

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
   }

   if (load_controller_thread.joinable())
      load_controller_thread.join();

   // TODO: add correct time (i.e. time required for the whole load funcition)
   time_since_sim_started += sim_period;

   return true;
}

bool MujocoRos2ControlPlugin::unload_controllers()
{
   bool unload_thread_finished = false;
   auto unload_controllers = [this, &unload_thread_finished]() {
      std::vector<std::string> no_controllers = {};
      if (controllers_to_load_name_.empty() == false)
      {
         rclcpp::Duration timeout = rclcpp::Duration(5, 0);
         if (controller_manager_->switch_controller(no_controllers, controllers_to_load_name_, 2, false, timeout) == controller_interface::return_type::ERROR)
         {
            RCLCPP_ERROR(controller_manager_->get_logger(), "Failed to deactivate controllers");
            return;
         }
      }

      unload_thread_finished = true;
   };
   std::thread unload_controller_thread = std::thread(unload_controllers);

   rclcpp::Duration sim_period = rclcpp::Duration(1, 0);
   while (unload_thread_finished != true)
   {
      controller_manager_->update(time_since_sim_started, sim_period);

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
   }

   if (unload_controller_thread.joinable())
      unload_controller_thread.join();

   // TODO: add correct time (i.e. time required for the whole load funcition)
   time_since_sim_started += sim_period;

   return true;
}

bool MujocoRos2ControlPlugin::launch_controllers()
{
   // Launching ros controller spawner launch file
   std::string launch_file_path = "/home/niccolo-hiro/ros/ur_mujoco_ws/install/ur_hiro_mujoco/share/ur_hiro_mujoco/launch/";
   std::string launch_file_name = "spawn_controllers.launch.py";

   std::string controller_to_start_arg = " controllers_to_start:=[";
   for (std::string controller_name : controllers_to_load_name_)
      controller_to_start_arg += "\\'" + controller_name + "\\',";
   controller_to_start_arg += "]";

   std::string controller_file_pkg_arg = " controller_file_pkg:=ur_hiro_bringup";

   std::string args = controller_to_start_arg + controller_file_pkg_arg;

   std::string command = launch_file_path + launch_file_name + args;
   std::string ros2_command = "ros2 launch " + command;

   int result = system(ros2_command.c_str());
   if (result == -1)
   {
      RCLCPP_ERROR(node_->get_logger(), "Failed to execute ros2 launch command");
      return false;
   }

   return true;
}

}  // namespace mujoco_ros2_control
