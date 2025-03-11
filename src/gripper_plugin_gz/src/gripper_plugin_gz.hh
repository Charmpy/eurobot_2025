#ifndef GRIPPER_PLUGIN_GZ_HPP
#define GRIPPER_PLUGIN_GZ_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gz/common/Console.hh>

namespace gripper_plugin_gz
{
class GripperPlugin : public gz::sim::System,
                      public gz::sim::ISystemConfigure
{
public:
  GripperPlugin();
  ~GripperPlugin() override = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

private:
  void OnGripperCommand(const std_msgs::msg::String::SharedPtr msg);

  gz::sim::Model model_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string gripper_link_name_;
  gz::sim::EntityComponentManager* ecm_; // Declare ecm_ here
};
}  // namespace gripper_plugin_gz

#endif  // GRIPPER_PLUGIN_GZ_HPP