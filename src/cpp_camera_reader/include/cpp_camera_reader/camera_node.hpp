#include "rclcpp/rclcpp.hpp"
#include <gst/gst.h>
#include <image_transport/image_transport.hpp>

class CameraGStreamerNode : public rclcpp::Node
{
public:
    explicit CameraGStreamerNode();  // Убрали параметр NodeOptions
    ~CameraGStreamerNode();

private:
    void initialize_gstreamer();
    void cleanup_gstreamer();
    
    GstElement* pipeline_;
    std::string camera_serial_;
    bool running_;
    
    image_transport::Publisher image_pub_;
};