#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <random>
#include <vector>

class ObjectSimPub : public rclcpp::Node {
    public:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr column_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr board_pub_;
        rclcpp::TimerBase::SharedPtr column_timer_;
        rclcpp::TimerBase::SharedPtr board_timer_;
        int change_position_column = 50;
        int change_position_board = 90;
        geometry_msgs::msg::PoseArray column_message;
        geometry_msgs::msg::PoseArray board_message;
        int mode = 0;
        std::vector<double> column_pos_x {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 1.35, 1.3, 1.25, 1.2, 1.65, 1.7, 1.75, 1.8, 0.85, 0.8, 0.75, 0.7, 2.15, 2.2, 2.25, 2.3, 0.85, 0.8, 0.75, 0.7, 2.15, 2.2, 2.25, 2.3};
        std::vector<double> column_pos_y {0.75, 0.7, 0.65, 0.6, 1.75, 1.7, 1.65, 1.6, 0.75, 0.7, 0.65, 0.6, 1.75, 1.7, 1.65, 1.6, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8};
        std::vector<double> board_pos_x {};
        std::vector<double> board_pos_y { /* fill with desired values */ };
        std::vector<double> board_orientation { /* fill with desired values */ };
        ObjectSimPub() : Node("object_sim_pub") {
            column_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/global_center_poses/column", 100);
            board_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/global_center_poses/platform", 100);
            column_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::column_timer_callback, this));
            board_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::board_timer_callback, this));
        }
    private:
        void column_timer_callback() {
            if(mode == 0){
                if(change_position_column == 0){
                    column_message = geometry_msgs::msg::PoseArray();
                    column_message.header.stamp = this->now();
                    column_message.header.frame_id = "column";
                    for(int i = 0; i < 5; i++){
                        auto column_pose = generate_random_pose();
                        column_message.poses.push_back(column_pose);
                    }
                    column_pub_->publish(column_message);
                    change_position_column = 50;
                }
                else column_pub_->publish(column_message);
                change_position_column--;
            }
            else if(mode == 1){
                column_message = geometry_msgs::msg::PoseArray();
                column_message.header.stamp = this->now();
                column_message.header.frame_id = "column";
                for(int i = 0; i < 40; i++){    
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = column_pos_x[i];
                    pose.position.y = column_pos_y[i];
                    column_message.poses.push_back(pose);
                }
                column_pub_->publish(column_message);
            }
        }
        void board_timer_callback() {
            if(mode == 0){
                if(change_position_board == 0){
                    board_message = geometry_msgs::msg::PoseArray();
                    board_message.header.stamp = this->now();
                    board_message.header.frame_id = "board";
                    for(int i = 0; i < 5; i++){
                        auto board_pose = generate_random_pose();
                        board_message.poses.push_back(board_pose);
                    }
                    board_pub_->publish(board_message);
                    change_position_board = 90;
                }
                else board_pub_->publish(board_message);
                change_position_board--;
            }
            else if(mode == 1){

            }
        }

        geometry_msgs::msg::Pose generate_random_pose(){
            geometry_msgs::msg::Pose pose;

            // Random engines and distributions for positions and yaw
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dist_x(0.0, 3.0);
            std::uniform_real_distribution<double> dist_y(0.0, 2.0);
            std::uniform_real_distribution<double> dist_yaw(0.0, 2*M_PI);
            
            // Generate random position
            pose.position.x = dist_x(gen);
            pose.position.y = dist_y(gen);
            
            // Generate random orientation using a random yaw
            double yaw = dist_yaw(gen);
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(yaw / 2.0);
            pose.orientation.w = std::cos(yaw / 2.0);
            
            return pose;
        }
           
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSimPub>());
    rclcpp::shutdown();
    return 0;
}