#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <gz/msgs.hh>
#include <gz/transport.hh>


using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotCMDVelSubscriber : public rclcpp::Node
{
  public:
    RobotCMDVelSubscriber()
    : Node("omni_gz_con")
    {      
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&RobotCMDVelSubscriber::topic_callback, this, _1));

      timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotCMDVelSubscriber::broadcast_timer_callback, this));
      
      node = std::make_shared<gz::transport::Node>();

      std::string topic = "/model/my_bot/cmd_vel";
      gz_pub = std::make_shared<gz::transport::Node::Publisher>(node->Advertise<gz::msgs::Twist>(topic)); // Создаём Publisher

      std::string odom_topic = "/odom";      
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000); // Создаём Odom Publisher
      
      odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);    

      x = 1.0;
      y = -0.2;
      w = 0.0;

      vel_x = 0.0;
      vel_y = 0.0;
      vel_w = 0.0;

      current_time = this->now().nanoseconds();
      prev_time = this->now().nanoseconds();
      
      broadcast_timer_callback();

    }
    mutable double vel_x;
    mutable double vel_y;
    mutable double vel_w;

    double x;
    double y;
    double w;

    double current_time;
    double prev_time;

    double angle = 0.0;
    std::shared_ptr<gz::transport::Node> node;  // Узел Gazebo Transport
    std::shared_ptr<gz::transport::Node::Publisher> gz_pub;
    // std::shared_ptr<gz::transport::Node::Publisher>  gz_pub; 

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;    
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  private:

    void broadcast_timer_callback()
    {
        nav_msgs::msg::Odometry odom_msg;
        
        current_time = this->now().nanoseconds();
        double d_time = current_time - prev_time;

        // RCLCPP_INFO(this->get_logger(), "At time '%f' i see: '%f', '%f', '%f'", current_time, vel_x, vel_y, vel_w);
        
        auto d_w = vel_w * d_time / std::pow(10,9);
        w += d_w;

        auto d_x = vel_x * d_time / std::pow(10,9) * std::cos(w) + vel_y * d_time / std::pow(10,9) * std::sin(w);
        auto d_y = vel_y * d_time / std::pow(10,9) * std::cos(w) + vel_x * d_time / std::pow(10,9) * std::sin(w);        
        x += d_x;
        y += d_y;
        
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, w);   
        // quaternion.normalize();        

        //next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;        
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.01;        
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = vel_y;
        odom.twist.twist.linear.z = 0.01;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vel_w;

        //publish the message             
        odom_pub -> publish(odom);

        //first, we'll publish the transform over tf        
        geometry_msgs::msg::TransformStamped odom_trans;
        // odom_trans.header.stamp = odom.header.stamp;
        odom_trans.header.stamp = this->now();        
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.01;        
        odom_trans.transform.rotation.x = quaternion.x();
        odom_trans.transform.rotation.y = quaternion.y();
        odom_trans.transform.rotation.z = quaternion.z();
        odom_trans.transform.rotation.w = quaternion.w();

        //send the transform        
        odom_broadcaster->sendTransform(odom_trans);

        prev_time = current_time;
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        // double vel_x = msg->linear.x;
        // double vel_y = msg->linear.y;
        // double vel_w = msg->angular.z; 
        vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_w = msg->angular.z;            

        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", vel_x);
        RCLCPP_INFO(this->get_logger(), "At time '%f' i see: '%f', '%f', '%f'", current_time, vel_x, vel_y, vel_w);


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
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotCMDVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}