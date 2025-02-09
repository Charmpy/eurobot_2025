#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include <gz/msgs.hh>
#include <gz/transport.hh>


using std::placeholders::_1;

class RobotCMDVelSubscriber : public rclcpp::Node
{
  public:
    RobotCMDVelSubscriber()
    : Node("omni_gz_con")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&RobotCMDVelSubscriber::topic_callback, this, _1));
      
      node = std::make_shared<gz::transport::Node>();

      std::string topic = "/model/my_bot/cmd_vel";
      gz_pub = std::make_shared<gz::transport::Node::Publisher>(node->Advertise<gz::msgs::Twist>(topic)); // Создаём Publisher

    }
    double angle = 0.0;
    std::shared_ptr<gz::transport::Node> node;  // Узел Gazebo Transport
    std::shared_ptr<gz::transport::Node::Publisher> gz_pub;
    // std::shared_ptr<gz::transport::Node::Publisher>  gz_pub; 
  private:


    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        double vel_x = msg->linear.x;
        double vel_y = msg->linear.y;
        double vel_w = msg->angular.z;    

        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", vel_x);


        // Prepare the message.
        gz::msgs::Twist gz_msg;
      //  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", vel_x);
        gz::msgs::Vector3d* vel_lin = gz_msg.mutable_linear();
        // vel_lin->set_x(sin(angle) * vel_x + cos(angle) * vel_y);
        // vel_lin->set_y(sin(angle) * vel_y + cos(angle) * vel_x);
        vel_lin->set_x(vel_x);
        vel_lin->set_y(vel_y);

      //  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", vel_x);
        gz::msgs::Vector3d* vel_ang = gz_msg.mutable_angular();
        vel_ang->set_z(vel_w);
      //  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", gz_msg.linear().x());

      gz_pub -> Publish(gz_msg);


    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotCMDVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}