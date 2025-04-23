#include "custom_costmap_layer/custom_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace custom_costmap_layer
{
CustomLayer::CustomLayer() : radius_(0.5), has_updated_(false)
{
}

void CustomLayer::onInitialize()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock node");
    }
    RCLCPP_INFO(node->get_logger(), "Custom Layer initialized");

    // Инициализация параметров
    radius_param_ = rclcpp::Parameter("radius", 0.5);
    node->declare_parameter("radius", rclcpp::ParameterValue(0.5));
    radius_ = node->get_parameter("radius").get_value<double>();

    // Подписка на топик /custom_costmap_points типа Float64MultiArray
    subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/custom_costmap_points", 10, std::bind(&CustomLayer::points_callback, this, std::placeholders::_1));
}

void CustomLayer::points_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    auto node = node_.lock();
    if (!node) return;

    points_.clear();  // Очищаем старые точки
    // Копируем данные из Float64MultiArray в points_ как [x1, y1, x2, y2, ...]
    points_ = msg->data;
    has_updated_ = true;  // Отмечаем, что данные обновлены
    RCLCPP_INFO(node->get_logger(), "Получено %zu точек (паре: %zu)", points_.size(), points_.size() / 2);
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
    if (!has_updated_ && !points_.empty()) return;  // Обновляем только при новых данных

    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);
            double max_cost = 0.0;

            for (size_t idx = 0; idx < points_.size(); idx += 2) {
                if (idx + 1 < points_.size()) {
                    double px = points_[idx];
                    double py = points_[idx + 1];
                    double distance = std::hypot(wx - px, wy - py);  // Расстояние до центра круга

                    if (distance <= radius_) {
                        // Градиент стоимости: максимум в центре, убывает к краям
                        double cost = (1.0 - (distance / radius_)) * nav2_costmap_2d::LETHAL_OBSTACLE;
                        max_cost = std::max(max_cost, cost);
                    }
                }
            }

            if (max_cost > 0) {
                unsigned char current_cost = master_grid.getCost(i, j);
                unsigned char new_cost = static_cast<unsigned char>(std::min(max_cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)));
                master_grid.setCost(i, j, std::max(current_cost, new_cost));
            }
        }
    }

    has_updated_ = false;  // Сбрасываем флаг после обновления
}

// Реализация чисто виртуальных методов
void CustomLayer::reset()
{
    points_.clear();  // Сбрасываем точки при вызове reset
    has_updated_ = false;
    RCLCPP_INFO(node_.lock()->get_logger(), "Layer reset");
}

bool CustomLayer::isClearable()
{
    return false;  // Слой не поддерживает очистку
}

}  // namespace custom_costmap_layer

PLUGINLIB_EXPORT_CLASS(custom_costmap_layer::CustomLayer, nav2_costmap_2d::Layer)