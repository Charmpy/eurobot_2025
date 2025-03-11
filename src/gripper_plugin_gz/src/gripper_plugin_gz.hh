#ifndef DYNAMIC_DETACHABLE_JOINT_HPP
#define DYNAMIC_DETACHABLE_JOINT_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <gz/common/Console.hh>

namespace gripper_plugin_gz
{
class DynamicDetachableJoint : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate
{
public:
  DynamicDetachableJoint();
  ~DynamicDetachableJoint() override = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

private:
  gz::sim::Model model_;                                    // The parent model (gripper)
  rclcpp::Node::SharedPtr ros_node_;                        // ROS 2 node for communication
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr attach_sub_; // Subscription to attach topic
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr detach_sub_;  // Subscription to detach topic
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;     // Publisher for state
  std::string attach_topic_;                                // Topic for attach commands
  std::string detach_topic_;                                // Topic for detach commands
  std::string output_topic_;                                // Topic for state output
  std::string grip_link_name_;                              // Name of the grip link in child models
  gz::sim::Entity parent_link_entity_;                      // Entity of the gripper's parent link
  gz::sim::Entity current_child_link_entity_;               // Entity of the currently attached child link
  gz::sim::Entity detachable_joint_entity_;                 // Entity of the detachable joint
  bool is_attached_;                                        // Whether the gripper is attached
  std::string requested_object_name_;                       // Name of the object to attach to
  bool attach_requested_;                                   // Flag for attachment request
  bool detach_requested_;                                   // Flag for detachment request
};
}  // namespace gripper_plugin_gz

#endif  // DYNAMIC_DETACHABLE_JOINT_HPP
