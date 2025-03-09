#ifndef CUSTOM_LAYER_HPP_
#define CUSTOM_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <vector>

namespace custom_costmap_layer
{
class CustomLayer : public nav2_costmap_2d::Layer
{
public:
  CustomLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override {}
  virtual bool isClearable() override { return false; }

private:
  double radius_;  // Радиус круга вокруг каждой точки
  std::vector<double> points_;  // Список координат [x1, y1, x2, y2, ...]
  rclcpp::Parameter radius_param_;  // Параметр для радиуса
  rclcpp::Parameter points_param_;  // Параметр для списка координат
};
}  // namespace custom_costmap_layer

#endif  // CUSTOM_LAYER_HPP_