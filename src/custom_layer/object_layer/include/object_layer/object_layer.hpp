#ifndef OBJECT_LAYER_HPP_
#define OBJECT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <deque>
#include <iostream>
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <algorithm>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"


namespace Object_costmap_plugin {
    class ObjectLayer : public nav2_costmap_2d::CostmapLayer {
        public:
            ObjectLayer(){}
            ~ObjectLayer(){}

            void onInitialize() override;
            void updateBounds(
                double robot_x, double robot_y, double robot_yaw, 
                double *min_x, double *min_y, double *max_x, double *max_y) override;
            void updateCosts(
                nav2_costmap_2d::Costmap2D &master_grid, 
                int min_i, int min_j, int max_i, int max_j) override;
            bool isClearable() override;
            void reset() override;
            
            void ExpandPointWithRectangle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, geometry_msgs::msg::PoseStamped object);
            void ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);
            // data processes
            void columnPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray);
            void boardPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray);
            void checkClear();        
        private:
            std::deque<geometry_msgs::msg::PoseStamped> columnList;
            std::deque<geometry_msgs::msg::PoseStamped> boardList;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr column_poseArray_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr board_poseArray_sub;
            int clearTimer;
            double column_inscribed_radius, board_inscribed_radius;
            double column_inflation_radius, board_inflation_radius;
            double cost_scaling_factor;
            double min_x_ = 0.0, min_y_ = 0.0, max_x_ = 3.0, max_y_ = 2.0;
            double board_width = 0.4, board_height = 0.1;

    };
} // namespace Object_costmap_plgin

#endif  // OBJECT_LAYER_HPP_