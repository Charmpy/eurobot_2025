#include "keepout_layer/keepout_layer.hpp"

namespace keepout_costmap_plugin {
    // KeepoutLayer class
    void KeepoutLayer::onInitialize() {
        RCLCPP_INFO(
            rclcpp::get_logger("KeepoutLayer"), 
            "Initializing KeepoutLayer");

        // Initialize the layer
        reset();

        // Get the node
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }
        
        // Declare the parameters
        declareParameter("keepout_zone_array", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.0}));

        declareParameter("inflation_length", rclcpp::ParameterValue(0.5));
        declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));

        // Get the parameters
        std::vector<double> keepout_zone_array_raw;
        node->get_parameter(name_ + "." + "keepout_zone_array", keepout_zone_array_raw);
        for(size_t i = 0; i+3 < keepout_zone_array_raw.size(); i += 4) {
            KeepoutZone zone;
            zone.x = keepout_zone_array_raw[i];
            zone.y = keepout_zone_array_raw[i + 1];
            zone.lengthX = keepout_zone_array_raw[i + 2];
            zone.lengthY = keepout_zone_array_raw[i + 3];
            keepout_zone_array_.push_back(zone);
        }

        node->get_parameter(name_ + "." + "inflation_length", inflation_length_);
        node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);

        keepout_zone_sub_ = node->create_subscription<std_msgs::msg::String>(
            "/keepout_zone", 10, std::bind(&KeepoutLayer::keepoutZoneCallback, this, std::placeholders::_1));
    }

    void KeepoutLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, 
        double *min_x, double *min_y, double *max_x, double *max_y) {

        // Update the bounds of the costmap
        *min_x = std::min(0.0, *min_x);
        *min_y = std::min(0.0, *min_y);
        *max_x = std::max(3.0, *max_x);
        *max_y = std::max(2.0, *max_y);
    }

    void KeepoutLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, 
        int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) {

        // Check if the layer is enabled
        if (!enabled_) {
            return;
        }

        auto node = node_.lock();
        node->get_parameter(name_ + "." + "inflation_length", inflation_length_);

        // Set the keepout zone
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
        SetKeepoutZone();
        updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    }

    bool KeepoutLayer::isClearable() {
        return true;
    }

    void KeepoutLayer::reset() {
        enabled_ = true;
        current_ = true;

        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
    }

    void KeepoutLayer::activate() {
        RCLCPP_INFO(
            rclcpp::get_logger("KeepoutLayer"), 
            "Activating KeepoutLayer");
    }
        
    void KeepoutLayer::deactivate() {
        RCLCPP_INFO(
            rclcpp::get_logger("KeepoutLayer"), 
            "Deactivating KeepoutLayer");
    }

    void KeepoutLayer::ExpandPointWithSquare(KeepoutZone zone, double max_cost, double inflation_radius, double cost_scaling_factor) {
        unsigned int mx, my;
    
        if (worldToMap(zone.x, zone.y, mx, my)) {
            double expand_length_x = zone.lengthX / 2.0 + inflation_radius;
            double expand_length_y = zone.lengthY / 2.0 + inflation_radius;
            double step = getResolution();
    
            for (double current_x = zone.x - expand_length_x; current_x <= zone.x + expand_length_x; current_x += step) {
                for (double current_y = zone.y - expand_length_y; current_y <= zone.y + expand_length_y; current_y += step) {
                    unsigned int cell_x, cell_y;
                    if (worldToMap(current_x, current_y, cell_x, cell_y)) {
                        double dx = std::max(0.0, fabs(current_x - zone.x) - zone.lengthX / 2.0);
                        double dy = std::max(0.0, fabs(current_y - zone.y) - zone.lengthY / 2.0);
                        double distance = hypot(dx, dy);
    
                        double cost = ceil(252 * exp(-cost_scaling_factor * distance));
                        cost = std::max(std::min(cost, max_cost), 0.0);
    
                        if (getCost(cell_x, cell_y) != nav2_costmap_2d::NO_INFORMATION) {
                            setCost(cell_x, cell_y, std::max((unsigned char)cost, getCost(cell_x, cell_y)));
                        } else {
                            setCost(cell_x, cell_y, cost);
                        }
                    }
                }
            }
        }
    }            

    void KeepoutLayer::SetKeepoutZone() {
        // Set the keepout zone based on the active keepout zones
        for(size_t i = 0; i < keepout_zone_array_.size(); i++) {
            if (i >= 26) {
                RCLCPP_WARN(rclcpp::get_logger("KeepoutLayer"), "Too many keepout zones (>26), ignoring extra zones.");
                break;
            }
            if(strchr(active_keepout_zones_.c_str(), 'A'+i) != NULL) {
                ExpandPointWithSquare(keepout_zone_array_[i], nav2_costmap_2d::LETHAL_OBSTACLE, inflation_length_, cost_scaling_factor_);
                // RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Active keepout zone %c", char('A'+i));
            }   
        }
    }

    void KeepoutLayer::keepoutZoneCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Get the active keepout zones
        active_keepout_zones_ = msg->data;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(keepout_costmap_plugin::KeepoutLayer, nav2_costmap_2d::Layer)