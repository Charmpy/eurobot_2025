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

  // Get grip link name from SDF, default to "grip_link"
  this->grip_link_name_ = sdf->Get<std::string>("grip_link", "grip_link").first;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  // Initialize ROS 2 node
  this->ros_node_ = rclcpp::Node::make_shared("dynamic_detachable_joint_" + std::to_string(entity));

  // Set up topic names based on model name
  std::string model_name = this->model_.Name(ecm);
  this->attach_topic_ = sdf->Get<std::string>("attach_topic", "/model/" + model_name + "/detachable_joint/attach").first;
  this->detach_topic_ = sdf->Get<std::string>("detach_topic", "/model/" + model_name + "/detachable_joint/detach").first;
  this->output_topic_ = sdf->Get<std::string>("output_topic", "/model/" + model_name + "/detachable_joint/state").first;
  // this->attach_topic_ = "/model/" + model_name + "/detachable_joint/attach";
  // this->detach_topic_ = "/model/" + model_name + "/detachable_joint/detach";
  // this->output_topic_ = "/model/" + model_name + "/detachable_joint/state";

  // Subscribe to attach topic (expects object name)
  this->attach_sub_ = this->ros_node_->create_subscription<std_msgs::msg::String>(
      this->attach_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (this->is_attached_)
        {
          gzdbg << "Gripper already attached, ignoring request for '"
                << msg->data << "'." << std::endl;
          return;
        }
        this->requested_object_name_ = msg->data;
        this->attach_requested_ = true;
      });

  // Subscribe to detach topic
  this->detach_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Empty>(
      this->detach_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        if (!this->is_attached_)
        {
          gzdbg << "Gripper not attached, ignoring detach request." << std::endl;
          return;
        }
        this->detach_requested_ = true;
      });

  // Set up state publisher
  this->state_pub_ = this->ros_node_->create_publisher<std_msgs::msg::String>(
      this->output_topic_, 10);

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

// Register the plugin with Gazebo Ignition
GZ_ADD_PLUGIN(
    gripper_plugin_gz::DynamicDetachableJoint,
    gz::sim::System,
    gripper_plugin_gz::DynamicDetachableJoint::ISystemConfigure,
    gripper_plugin_gz::DynamicDetachableJoint::ISystemPreUpdate
)