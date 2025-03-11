#include "gripper_plugin_gz.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/common/Console.hh>

namespace gripper_plugin_gz
{
GripperPlugin::GripperPlugin()
{
  // Initialize ROS 2 if not already initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
}

void GripperPlugin::Configure(const gz::sim::Entity& entity,
                              const std::shared_ptr<const sdf::Element>& sdf,
                              gz::sim::EntityComponentManager& ecm,
                              gz::sim::EventManager& /*eventMgr*/)
{
  // Initialize the model
  model_ = gz::sim::Model(entity);

  // Store the ecm pointer
  ecm_ = &ecm;

  // Create a ROS node
  ros_node_ = rclcpp::Node::make_shared("gripper_plugin_node");

  // Get the gripper link name from SDF (default: "gripper_link")
  gripper_link_name_ = sdf->Get<std::string>("gripper_link", "gripper_link").first;

  // Subscribe to the /gripper_command topic
  subscription_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/gripper_command", 10,
      std::bind(&GripperPlugin::OnGripperCommand, this, std::placeholders::_1));

  // Spin the ROS node in a separate thread
  std::thread([this]() { rclcpp::spin(ros_node_); }).detach();

  gzdbg << "Gripper plugin configured with gripper link: " << gripper_link_name_ << std::endl;
}

void GripperPlugin::OnGripperCommand(const std_msgs::msg::String::SharedPtr msg)
{
  if (!ecm_) {
    gzerr << "EntityComponentManager not initialized" << std::endl;
    return;
  }

  std::string object_name = msg->data;
  gzdbg << "Received command to grab object: " << object_name << std::endl;

  // Find the object in the world (must be a model)
  auto object_entities = ecm_->EntitiesByComponents(
      gz::sim::components::Name(object_name),
      gz::sim::components::Model());
  if (object_entities.empty()) {
    gzerr << "Object '" << object_name << "' not found or not a model" << std::endl;
    return;
  }
  auto object_entity = object_entities.front();

  // Get the gripper link entity
  auto gripper_link_entity = model_.LinkByName(*ecm_, gripper_link_name_);
  if (gripper_link_entity == gz::sim::kNullEntity) {
    gzerr << "Gripper link '" << gripper_link_name_ << "' not found in model" << std::endl;
    return;
  }

  // Get the gripper's pose
  auto gripper_pose_comp = ecm_->Component<gz::sim::components::Pose>(gripper_link_entity);
  if (!gripper_pose_comp) {
    gzerr << "Gripper link has no Pose component" << std::endl;
    return;
  }
  auto gripper_pose = gripper_pose_comp->Data();

  // Move the object to the gripper's pose
  auto object_pose_comp = ecm_->Component<gz::sim::components::Pose>(object_entity);
  if (object_pose_comp) {
    object_pose_comp->Data() = gripper_pose;
    ecm_->SetChanged(object_entity, gz::sim::components::Pose::typeId,
                     gz::sim::ComponentState::PeriodicChange);
    gzdbg << "Moved " << object_name << " to gripper position" << std::endl;
  } else {
    gzerr << "Object has no Pose component" << std::endl;
  }
}
}  // namespace gripper_plugin_gz

// Register the plugin with Gazebo Ignition 8
GZ_ADD_PLUGIN(
    gripper_plugin_gz::GripperPlugin,
    gz::sim::System,
    gripper_plugin_gz::GripperPlugin::ISystemConfigure
)
// GZ_ADD_PLUGIN_ALIAS(gripper_plugin_gz::GripperPlugin,
//     "gz::sim::systems::GripperPlugin")