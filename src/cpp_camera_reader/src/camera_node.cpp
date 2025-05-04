#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/qos.hpp"
#include <memory>
#include <string>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        // Initialize publisher with QoS settings
        rclcpp::QoS qos_profile(2);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos_profile);

        // Initialize GStreamer
        gst_init(NULL, NULL);
        
        // Create pipeline with tcambin source and appsink
        pipeline_ = gst_parse_launch(
            "tcambin name=source ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink", 
            NULL);
        
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline");
            return;
        }
        
        // Get the source element
        source_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
        if (!source_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get tcambin source");
            return;
        }

        // Set camera serial number if needed
        const char* serial = nullptr; // Set your camera serial here if needed
        if (serial != nullptr) {
            g_object_set(source_, "serial", serial, NULL);
        }

        // Get the appsink element
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsink");
            return;
        }
        
        // Configure appsink
        g_object_set(sink, "emit-signals", TRUE, "sync", FALSE, NULL);
        g_signal_connect(sink, "new-sample", G_CALLBACK(&CameraPublisher::new_sample_callback), this);
        gst_object_unref(sink);

        // Configure camera properties
        set_camera_property_string("ExposureAuto", "Off");
        set_camera_property_double("ExposureTime", 40000.0);
        set_camera_property_string("GainAuto", "Off");
        set_camera_property_double("Gain", 10.0);
        
        // Start pipeline
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera publisher started successfully");
    }
    
    ~CameraPublisher()
    {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        if (source_) {
            gst_object_unref(source_);
        }
    }
    
private:
    // Property setting functions for different types
    void set_camera_property_string(const std::string& name, const std::string& value)
    {
        if (!source_) return;
        
        GValue current = G_VALUE_INIT;
        g_value_init(&current, GST_TYPE_STRUCTURE);
        g_object_get_property(G_OBJECT(source_), "tcam-properties", &current);

        GstStructure* new_props = gst_structure_copy(gst_value_get_structure(&current));
        gst_structure_set(new_props, name.c_str(), G_TYPE_STRING, value.c_str(), NULL);

        GValue new_val = G_VALUE_INIT;
        g_value_init(&new_val, GST_TYPE_STRUCTURE);
        gst_value_set_structure(&new_val, new_props);
        g_object_set_property(G_OBJECT(source_), "tcam-properties", &new_val);

        g_value_unset(&new_val);
        gst_structure_free(new_props);
        g_value_unset(&current);

        RCLCPP_INFO(this->get_logger(), "Set property '%s'='%s'", name.c_str(), value.c_str());
    }

    void set_camera_property_double(const std::string& name, double value)
    {
        if (!source_) return;
        
        GValue current = G_VALUE_INIT;
        g_value_init(&current, GST_TYPE_STRUCTURE);
        g_object_get_property(G_OBJECT(source_), "tcam-properties", &current);

        GstStructure* new_props = gst_structure_copy(gst_value_get_structure(&current));
        gst_structure_set(new_props, name.c_str(), G_TYPE_DOUBLE, value, NULL);

        GValue new_val = G_VALUE_INIT;
        g_value_init(&new_val, GST_TYPE_STRUCTURE);
        gst_value_set_structure(&new_val, new_props);
        g_object_set_property(G_OBJECT(source_), "tcam-properties", &new_val);

        g_value_unset(&new_val);
        gst_structure_free(new_props);
        g_value_unset(&current);

        RCLCPP_INFO(this->get_logger(), "Set property '%s'=%.2f", name.c_str(), value);
    }

    void set_camera_property_int(const std::string& name, int value)
    {
        if (!source_) return;
        
        GValue current = G_VALUE_INIT;
        g_value_init(&current, GST_TYPE_STRUCTURE);
        g_object_get_property(G_OBJECT(source_), "tcam-properties", &current);

        GstStructure* new_props = gst_structure_copy(gst_value_get_structure(&current));
        gst_structure_set(new_props, name.c_str(), G_TYPE_INT, value, NULL);

        GValue new_val = G_VALUE_INIT;
        g_value_init(&new_val, GST_TYPE_STRUCTURE);
        gst_value_set_structure(&new_val, new_props);
        g_object_set_property(G_OBJECT(source_), "tcam-properties", &new_val);

        g_value_unset(&new_val);
        gst_structure_free(new_props);
        g_value_unset(&current);

        RCLCPP_INFO(this->get_logger(), "Set property '%s'=%d", name.c_str(), value);
    }

    void set_camera_property_bool(const std::string& name, bool value)
    {
        if (!source_) return;
        
        GValue current = G_VALUE_INIT;
        g_value_init(&current, GST_TYPE_STRUCTURE);
        g_object_get_property(G_OBJECT(source_), "tcam-properties", &current);

        GstStructure* new_props = gst_structure_copy(gst_value_get_structure(&current));
        gst_structure_set(new_props, name.c_str(), G_TYPE_BOOLEAN, value, NULL);

        GValue new_val = G_VALUE_INIT;
        g_value_init(&new_val, GST_TYPE_STRUCTURE);
        gst_value_set_structure(&new_val, new_props);
        g_object_set_property(G_OBJECT(source_), "tcam-properties", &new_val);

        g_value_unset(&new_val);
        gst_structure_free(new_props);
        g_value_unset(&current);

        RCLCPP_INFO(this->get_logger(), "Set property '%s'=%s", name.c_str(), value ? "true" : "false");
    }

    static GstFlowReturn new_sample_callback(GstElement* sink, CameraPublisher* node)
    {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) return GST_FLOW_ERROR;
        
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // Create OpenCV Mat from buffer (adjust resolution as needed)
            cv::Mat frame(cv::Size(2048, 1536), CV_8UC3, (void*)map.data, cv::Mat::AUTO_STEP);
            
            // Optional display
            // cv::imshow("Camera View", frame);
            // cv::waitKey(1);
            
            // Convert to ROS message
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
            msg->header.stamp = node->now();
            msg->header.frame_id = "camera";
            
            // Publish
            node->publisher_->publish(*msg);
            
            gst_buffer_unmap(buffer, &map);
        }
        
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    GstElement* pipeline_ = nullptr;
    GstElement* source_ = nullptr;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}