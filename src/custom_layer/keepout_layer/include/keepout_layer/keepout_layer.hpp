#ifndef KEEPOUT_LAYER_HPP_
#define KEEPOUT_LAYER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"

namespace keepout_costmap_plugin {
    struct KeepoutZone {
        double x;
        double y;
        double lengthX;
        double lengthY;
    };

    class KeepoutLayer : public nav2_costmap_2d::CostmapLayer {
        public:
            KeepoutLayer() {}
            ~KeepoutLayer() {}

            // Functions from Layers
            void onInitialize() override;
            void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y) override;
            void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
            bool isClearable() override;
            void reset() override;
            
            // Functions from Lifecycle Nodes
            void activate() override;
            void deactivate() override;

        private:  
            // Functions for costmap expansion
            void ExpandPointWithSquare(KeepoutZone Zone, double MaxCost, double InflationRadius, double CostScalingFactor);

            // Determine the keepout zone
            void SetKeepoutZone();

            // Rival pose subscibtion
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keepout_zone_sub_;
            void keepoutZoneCallback(const std_msgs::msg::String::SharedPtr msg);

            // Parameters
            std::vector<KeepoutZone> keepout_zone_array_;

            double inflation_length_;
            double cost_scaling_factor_;

            // Variables
            std::string active_keepout_zones_;
    };
}

#endif // KEEPOUT_LAYER_HPP_