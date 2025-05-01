#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


class UARTNode : public rclcpp::Node {
  public:
      UARTNode() : Node("uart_node") {
          // Получение параметров
          std::string device;
          int baudrate, bytesize, stopbits;
          std::string parity;
  
          this->declare_parameter("device", "/dev/ttyUSB0");
          this->declare_parameter("baudrate", 115200);
          this->declare_parameter("bytesize", 8);
          this->declare_parameter("parity", "none");
          this->declare_parameter("stopbits", 1);
  
          this->get_parameter("device", device);
          this->get_parameter("baudrate", baudrate);
          this->get_parameter("bytesize", bytesize);
          this->get_parameter("parity", parity);
          this->get_parameter("stopbits", stopbits);

          // Открываем UART
          uart_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
          if (uart_fd_ < 0) {
              RCLCPP_ERROR(this->get_logger(), "Не удалось открыть UART: %s", device.c_str());
              return;
          }
  
          // Настройка порта
          struct termios tty{};
          if (tcgetattr(uart_fd_, &tty) != 0) {
              RCLCPP_ERROR(this->get_logger(), "Ошибка получения настроек UART");
              return;
          }
  
          speed_t speed;
          switch (baudrate) {
              case 9600: speed = B9600; break;
              case 19200: speed = B19200; break;
              case 38400: speed = B38400; break;
              case 57600: speed = B57600; break;
              case 115200: speed = B115200; break;
              default:
                  RCLCPP_ERROR(this->get_logger(), "Неподдерживаемая скорость: %d", baudrate);
                  return;
          }
  
          cfsetospeed(&tty, speed);
          cfsetispeed(&tty, speed);
  
          // Биты данных
          tty.c_cflag &= ~CSIZE;
          switch (bytesize) {
              case 5: tty.c_cflag |= CS5; break;
              case 6: tty.c_cflag |= CS6; break;
              case 7: tty.c_cflag |= CS7; break;
              case 8: tty.c_cflag |= CS8; break;
              default:
                  RCLCPP_ERROR(this->get_logger(), "Неподдерживаемое количество бит данных: %d", bytesize);
                  return;
          }
  
          // Четность
          if (parity == "none") {
              tty.c_cflag &= ~PARENB;
          } else if (parity == "even") {
              tty.c_cflag |= PARENB;
              tty.c_cflag &= ~PARODD;
          } else if (parity == "odd") {
              tty.c_cflag |= PARENB;
              tty.c_cflag |= PARODD;
          } else {
              RCLCPP_ERROR(this->get_logger(), "Неподдерживаемая четность: %s", parity.c_str());
              return;
          }
  
          // Стоп-биты
          if (stopbits == 1) {
              tty.c_cflag &= ~CSTOPB;
          } else if (stopbits == 2) {
              tty.c_cflag |= CSTOPB;
          } else {
              RCLCPP_ERROR(this->get_logger(), "Неподдерживаемое количество стоп-бит: %d", stopbits);
              return;
          }
  
          tty.c_cflag |= (CLOCAL | CREAD);
          tty.c_iflag &= ~IGNBRK;
          tty.c_lflag = 0;
          tty.c_oflag = 0;
          tty.c_cc[VMIN]  = 1;
          tty.c_cc[VTIME] = 1;
  
          if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
              RCLCPP_ERROR(this->get_logger(), "Ошибка применения настроек UART");
              return;
          }
  
          // Подписка на входящие сообщения для отправки в UART
          sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&UARTNode::topic_callback, this, _1));
      
  
          // Паблишер для вывода из UART
          pub_ = this->create_publisher<std_msgs::msg::String>("uart_rx", 10);
  
          // Поток для чтения из UART
          uart_thread_ = std::thread([this]() { uart_read_loop(); });


        std::string odom_topic = "/odom";      
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000); // Создаём Odom Publisher
        odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);  

        timer_ = this->create_wall_timer(
            100ms, std::bind(&UARTNode::broadcast_timer_callback, this));

        current_time = this->now().nanoseconds();
        prev_time = this->now().nanoseconds();

        // reset_odometry_sub = this->create_subscription<std_msgs::msg::String>(
        //     "reset_odom", 10, std::bind(&UARTNode::reset_odom, this, _1));

            

        x = 1.0;
        y = -0.3;
        w = 0.111;

        v_x = 0.0;
        v_y = 0.0;
        v_w = 0.0;

      }


  
      ~UARTNode() override {
          running_ = false;
          if (uart_thread_.joinable()) {
              uart_thread_.join();
          }
          if (uart_fd_ >= 0) {
              close(uart_fd_);
          }
      }
    mutable double vel_x;
    mutable double vel_y;
    mutable double vel_w;

    double angle = 0.0;

    double x, y, w, v_x, v_y, v_w;
    double move_x, move_y, move_w;
    double real_x, real_y, real_w;

    double current_time;
    double prev_time;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;    
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  private:
      int uart_fd_;
      std::atomic<bool> running_{true};
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      std::thread uart_thread_;
      rclcpp::TimerBase::SharedPtr timer_;


      void broadcast_timer_callback()
      {
        std::ostringstream oss;
        oss << "odom \r";
        
        std::string result = oss.str();
        // RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
        write(uart_fd_, result.c_str(), result.size());
      }

      void reset_odom(const std_msgs::msg::String::SharedPtr msg) const
      {

        // std::istringstream(msg.data) >>real_x >> real_y >> real_w;

        // oss << real_x <<"|" << real_y << "|" << real_w;
        // std::string result = oss.str();
        // RCLCPP_INFO(this->get_logger(), "%s", result.c_str());

      }

      std::string read_until_delim(int fd) {
        std::string result;
        char ch;
        while (true) {
            int n = read(fd, &ch, 1);  // читаем по 1 байту
            if (n > 0) {
                if (ch == '\r') {
                    continue;
                } 
                if (ch == '\n') break;
                result += ch;
                
            } else if (n == 0) {
                // Нет данных, можно добавить таймаут или sleep
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                // Ошибка чтения
                break;
            }
        }
        return result;
        }


      void make_odom_from_str(std::string in_str) {

        std::istringstream(in_str) >> v_x >> v_y >> v_w >> x >> y >> w;
        // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", x, y, w, v_x, v_y, v_w);
        
        nav_msgs::msg::Odometry odom_msg;
        
        current_time = this->now().nanoseconds();
        double d_time = current_time - prev_time;

        // RCLCPP_INFO(this->get_logger(), "At time '%f' i see: '%f', '%f', '%f'", current_time, vel_x, vel_y, vel_w);
        
        auto d_w = v_w * d_time / std::pow(10,9);
        w += d_w;

        auto d_x = v_x * d_time / std::pow(10,9) * std::cos(w) + v_y * d_time / std::pow(10,9) * std::sin(w);
        auto d_y = v_y * d_time / std::pow(10,9) * std::cos(w) + v_x * d_time / std::pow(10,9) * std::sin(w);        
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
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;
        odom.twist.twist.linear.z = 0.01;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = v_w;

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
  
      void uart_read_loop() {
        //   char buf[256];
          while (running_) {
            std::string line = read_until_delim(uart_fd_);  // или другой символ
            if (!line.empty()) {
                make_odom_from_str(line);
                std_msgs::msg::String msg;
                msg.data = line;
                pub_->publish(msg);
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
      }

      void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
      {
        vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_w = msg->angular.z;            

        std::ostringstream oss;
        oss << "set_body_vel " << (int) (vel_x * 1000) << " " << (int) (vel_y * 1000) << " " << (int) (vel_w * 1000) <<"\r";
        
        

        std::string result = oss.str();
        RCLCPP_INFO(this->get_logger(), "%s", result.c_str());

        write(uart_fd_, result.c_str(), result.size());
      }
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UARTNode>());
  rclcpp::shutdown();
  return 0;
}
