#ifndef CUSTOM_COSTMAP_LAYER_CUSTOM_LAYER_HPP_
#define CUSTOM_COSTMAP_LAYER_CUSTOM_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace custom_costmap_layer
{
class CustomLayer : public nav2_costmap_2d::Layer
{
public:
    CustomLayer();

    void onInitialize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                     double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;
    void reset() override;
    bool isClearable() override;

private:
    void points_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    double radius_;
    rclcpp::Parameter radius_param_;
    std::vector<double> points_;
    bool has_updated_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};
}  // namespace custom_costmap_layer

#endif  // CUSTOM_COSTMAP_LAYER_CUSTOM_LAYER_HPP_