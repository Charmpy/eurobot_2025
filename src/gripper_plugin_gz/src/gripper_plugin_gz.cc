#include "gripper_plugin_gz.hh"
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace gripper_plugin_gz
{
  DynamicDetachableJoint::DynamicDetachableJoint()
  : model_(),
    ros_node_(nullptr),
    attach_sub_(nullptr),
    detach_sub_(nullptr),
    state_pub_(nullptr),
    attach_topic_(),
    detach_topic_(),
    output_topic_(),
    grip_link_name_(),
    parent_link_entity_(gz::sim::kNullEntity),
    current_child_link_entity_(gz::sim::kNullEntity),
    detachable_joint_entity_(gz::sim::kNullEntity),  // Matches declaration order
    is_attached_(false),
    requested_object_name_(),
    attach_requested_(false),
    detach_requested_(false)                         // Matches declaration order
{
}

void DynamicDetachableJoint::Configure(const gz::sim::Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager& /*eventMgr*/)
{
  this->model_ = gz::sim::Model(entity);
  if (!this->model_.Valid(ecm))
  {
    gzerr << "DynamicDetachableJoint must be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Get parent link from SDF
  if (sdf->HasElement("parent_link"))
  {
    auto parentLinkName = sdf->Get<std::string>("parent_link");
    this->parent_link_entity_ = this->model_.LinkByName(ecm, parentLinkName);
    if (this->parent_link_entity_ == gz::sim::kNullEntity)
    {
      gzerr << "Link '" << parentLinkName << "' not found in model '"
            << this->model_.Name(ecm) << "'. Failed to initialize." << std::endl;
      return;
    }
  }
  else
  {
    gzerr << "'parent_link' is required for DynamicDetachableJoint. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Get the gripper's pose
  auto gripper_pose_comp = ecm_->Component<gz::sim::components::Pose>(gripper_link_entity);
  if (!gripper_pose_comp) {
    gzerr << "Gripper link has no Pose component" << std::endl;
    return;
  }
  auto gripper_pose = gripper_pose_comp->Data();

  // Spin ROS node in a separate thread
  std::thread([this]() { rclcpp::spin(this->ros_node_); }).detach();
}

void DynamicDetachableJoint::PreUpdate(const gz::sim::UpdateInfo& /*info*/,
                                       gz::sim::EntityComponentManager& ecm)
{
  // Handle attachment request
  if (this->attach_requested_ && !this->is_attached_)
  {
    // Find the model with the requested name
    auto modelEntity = ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(this->requested_object_name_));
    if (modelEntity != gz::sim::kNullEntity)
    {
      // Find the grip link in the model
      auto gripLinkEntity = ecm.EntityByComponents(
          gz::sim::components::Link(),
          gz::sim::components::ParentEntity(modelEntity),
          gz::sim::components::Name(this->grip_link_name_));
      if (gripLinkEntity != gz::sim::kNullEntity)
      {
        // Create a detachable joint
        this->detachable_joint_entity_ = ecm.CreateEntity();
        ecm.CreateComponent(
            this->detachable_joint_entity_,
            gz::sim::components::DetachableJoint(
                {this->parent_link_entity_, gripLinkEntity, "fixed"}));
        this->current_child_link_entity_ = gripLinkEntity;
        this->is_attached_ = true;
        this->attach_requested_ = false;

        // Publish state
        std_msgs::msg::String msg;
        msg.data = "attached to " + this->requested_object_name_;
        this->state_pub_->publish(msg);
        gzdbg << "Attached to '" << this->requested_object_name_ << "'." << std::endl;
      }
      else
      {
        gzwarn << "Link '" << this->grip_link_name_ << "' not found in model '"
               << this->requested_object_name_ << "'." << std::endl;
      }
    }
    else
    {
      gzwarn << "Model '" << this->requested_object_name_ << "' not found." << std::endl;
    }
    this->attach_requested_ = false; // Reset request even if failed
  }

  // Handle detachment request
  if (this->detach_requested_ && this->is_attached_)
  {
    ecm.RequestRemoveEntity(this->detachable_joint_entity_);
    this->detachable_joint_entity_ = gz::sim::kNullEntity;
    this->current_child_link_entity_ = gz::sim::kNullEntity;
    this->is_attached_ = false;
    this->detach_requested_ = false;

    // Publish state
    std_msgs::msg::String msg;
    msg.data = "detached";
    this->state_pub_->publish(msg);
    gzdbg << "Detached." << std::endl;
  }
}
}  // namespace gripper_plugin_gz

// Register the plugin with Gazebo Ignition 8
GZ_ADD_PLUGIN(
    gripper_plugin_gz::DynamicDetachableJoint,
    gz::sim::System,
    gripper_plugin_gz::DynamicDetachableJoint::ISystemConfigure,
    gripper_plugin_gz::DynamicDetachableJoint::ISystemPreUpdate
)
// GZ_ADD_PLUGIN_ALIAS(gripper_plugin_gz::GripperPlugin,
//     "gz::sim::systems::GripperPlugin")