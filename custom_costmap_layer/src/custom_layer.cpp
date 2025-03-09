#include "custom_costmap_layer/custom_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace custom_costmap_layer
{
CustomLayer::CustomLayer() : radius_(0.5)  // По умолчанию радиус 0.5 м
{
}

void CustomLayer::onInitialize()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "Custom Layer initialized");

  // Инициализация параметров
  radius_param_ = rclcpp::Parameter("radius", 0.5);  // Радиус по умолчанию
  points_param_ = rclcpp::Parameter("points", std::vector<double>{0.5, 0.5, 1.0, 1.0});  // По умолчанию две точки: [x1, y1, x2, y2]
  node->declare_parameter("radius", rclcpp::ParameterValue(0.5));
  node->declare_parameter("points", rclcpp::ParameterValue(std::vector<double>{0.5, 0.5, 1.0, 1.0}));

  // Получаем значения параметров
  radius_ = node->get_parameter("radius").get_value<double>();
  points_ = node->get_parameter("points").get_value<std::vector<double>>();
}

void CustomLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (size_t i = 0; i < points_.size(); i += 2) {
    if (i + 1 < points_.size()) {
      double px = points_[i];
      double py = points_[i + 1];
      *min_x = std::min(*min_x, px - radius_);
      *min_y = std::min(*min_y, py - radius_);
      *max_x = std::max(*max_x, px + radius_);
      *max_y = std::max(*max_y, py + radius_);
    }
  }
}

void CustomLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                              int min_i, int min_j, int max_i, int max_j)
{
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      for (size_t idx = 0; idx < points_.size(); idx += 2) {
        if (idx + 1 < points_.size()) {
          double px = points_[idx];
          double py = points_[idx + 1];
          double distance = std::hypot(wx - px, wy - py);  // Расстояние до центра круга
          if (distance <= radius_) {
            master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);  // 254
            break;  // Если точка внутри круга, больше не проверяем
          }
        }
      }
    }
  }
}
}  // namespace custom_costmap_layer

PLUGINLIB_EXPORT_CLASS(custom_costmap_layer::CustomLayer, nav2_costmap_2d::Layer)