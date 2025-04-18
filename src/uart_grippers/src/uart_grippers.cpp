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
      UARTNode() : Node("grippers_node") {
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
          sub_ = this->create_subscription<std_msgs::msg::String>(
            "grippers", 10, std::bind(&UARTNode::topic_callback, this, _1));
      
  
          // Паблишер для вывода из UART
          pub_ = this->create_publisher<std_msgs::msg::String>("grippers_rx", 10);
  
          // Поток для чтения из UART
          uart_thread_ = std::thread([this]() { uart_read_loop(); });

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

  private:
      int uart_fd_;
      std::atomic<bool> running_{true};
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      std::thread uart_thread_;
      rclcpp::TimerBase::SharedPtr timer_;
  
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

      void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
      {
        String inp_msg = msg.data;
        if inp_msg.compare("build") == 0 {
            std::ostringstream oss;
            oss << "build\r";
            std::string result = oss.str();
            RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
            write(uart_fd_, result.c_str(), result.size());
        }
        if inp_msg.compare("compile") == 0 {
            std::ostringstream oss;
            oss << "compile\r";
            std::string result = oss.str();
            RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
            write(uart_fd_, result.c_str(), result.size());
        }
        if inp_msg.compare("start_state") == 0 {
            std::ostringstream oss;
            oss << "start_state\r";
            std::string result = oss.str();
            RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
            write(uart_fd_, result.c_str(), result.size());
        }

      }
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UARTNode>());
  rclcpp::shutdown();
  return 0;
}
