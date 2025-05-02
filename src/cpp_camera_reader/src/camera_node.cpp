// #include "cpp_camera_reader/camera_node.hpp"
// #include <memory>
// #include <string>

// CameraGStreamerNode::CameraGStreamerNode()
// : Node("camera_gstreamer_node"), pipeline_(nullptr), running_(false)
// {
//     // Получаем параметры
//     this->declare_parameter<std::string>("serial", "");
//     camera_serial_ = this->get_parameter("serial").as_string();
    
//     // Инициализируем GStreamer
//     initialize_gstreamer();
    
//     // Создаем publisher для изображений
//     auto image_pub = image_transport::create_publisher(this, "image_raw");
//     image_pub_ = image_pub;
    
//     RCLCPP_INFO(this->get_logger(), "Camera GStreamer node initialized");
// }

// CameraGStreamerNode::~CameraGStreamerNode()
// {
//     cleanup_gstreamer();
// }

// void CameraGStreamerNode::initialize_gstreamer()
// {
//     // Инициализация GStreamer
//     gst_init(NULL, NULL);
//     gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    
//     // Создаем pipeline
//     GError* err = NULL;
//     std::string pipeline_str = "tcambin name=source ! videoconvert ! appsink name=appsink";
//     pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    
//     if (err) {
//         RCLCPP_ERROR(this->get_logger(), "GStreamer error: %s", err->message);
//         g_error_free(err);
//         return;
//     }
    
//     if (!pipeline_) {
//         RCLCPP_ERROR(this->get_logger(), "Could not create pipeline");
//         return;
//     }
    
//     // Устанавливаем серийный номер камеры, если указан
//     if (!camera_serial_.empty()) {
//         GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
//         g_object_set(source, "serial", camera_serial_.c_str(), NULL);
//         gst_object_unref(source);
//     }
    
//     // Запускаем pipeline
//     if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
//         gst_object_unref(pipeline_);
//         pipeline_ = nullptr;
//         return;
//     }
    
//     running_ = true;
//     RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started");
// }

// void CameraGStreamerNode::cleanup_gstreamer()
// {
//     if (pipeline_) {
//         gst_element_set_state(pipeline_, GST_STATE_NULL);
//         gst_object_unref(pipeline_);
//         pipeline_ = nullptr;
//     }
//     running_ = false;
// }

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraGStreamerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



// -----------------------------------------------------
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <gst/gst.h>
// #include <gst/app/gstappsink.h>
// #include <memory>
// #include <string>

// class CameraPublisher : public rclcpp::Node
// {
// public:
//     CameraPublisher() : Node("camera_publisher"), pipeline_(nullptr)
//     {
//         // Инициализация publisher
//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
//             "camera/image_raw", 
//             rclcpp::SensorDataQoS().reliable());
        
//         // Параметры
//         this->declare_parameter<std::string>("serial", "");
//         camera_serial_ = this->get_parameter("serial").as_string();
        
//         // Инициализация GStreamer
//         if (!initialize_gstreamer()) {
//             RCLCPP_ERROR(get_logger(), "Failed to initialize GStreamer pipeline");
//             return;
//         }
        
//         // Таймер для проверки состояния pipeline
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             [this]() { check_pipeline_state(); });
        
//         RCLCPP_INFO(get_logger(), "Camera publisher node started");
//     }

//     ~CameraPublisher()
//     {
//         cleanup_gstreamer();
//     }

// private:
//     bool initialize_gstreamer()
//     {
//         // Инициализация GStreamer
//         if (!gst_is_initialized()) {
//             gst_init(NULL, NULL);
//         }
        
//         // Создаем pipeline
//         std::string pipeline_str = "tcambin ! videoconvert ! ximagesink";
//         // if (!camera_serial_.empty()) {
//         //     pipeline_str += " serial=" + camera_serial_;
//         // }

//         GError* error = NULL;
//         pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        
//         if (error) {
//             RCLCPP_ERROR(get_logger(), "Pipeline error: %s", error->message);
//             g_error_free(error);
//             return false;
//         }
        
//         if (!pipeline_) {
//             RCLCPP_ERROR(get_logger(), "Failed to create pipeline");
//             return false;
//         }
        
//         // Получаем appsink
//         GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
//         if (!appsink) {
//             RCLCPP_ERROR(get_logger(), "Failed to get appsink element");
//             gst_object_unref(pipeline_);
//             pipeline_ = nullptr;
//             return false;
//         }
        
//         // Настраиваем appsink
//         g_object_set(appsink,
//             "emit-signals", TRUE,
//             "sync", FALSE,
//             "max-buffers", 1,
//             "drop", TRUE,
//             NULL);
        
//         // Подключаем callback
//         g_signal_connect(appsink, "new-sample", 
//             G_CALLBACK(+[](GstElement* sink, CameraPublisher* self) {
//                 return self->handle_new_sample(sink);
//             }), this);
        
//         gst_object_unref(appsink);
        
//         // Запускаем pipeline
//         GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
//         if (ret == GST_STATE_CHANGE_FAILURE) {
//             RCLCPP_ERROR(get_logger(), "Failed to start pipeline");
//             print_pipeline_errors();
//             gst_object_unref(pipeline_);
//             pipeline_ = nullptr;
//             return false;
//         }
        
//         return true;
//     }
    
//     GstFlowReturn handle_new_sample(GstElement* appsink)
//     {
//         if (!appsink) {
//             return GST_FLOW_ERROR;
//         }
        
//         // Получаем sample
//         GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
//         if (!sample) {
//             return GST_FLOW_ERROR;
//         }
        
//         // Получаем буфер
//         GstBuffer* buffer = gst_sample_get_buffer(sample);
//         if (!buffer) {
//             gst_sample_unref(sample);
//             return GST_FLOW_ERROR;
//         }
        
//         // Создаем ROS сообщение
//         auto msg = std::make_shared<sensor_msgs::msg::Image>();
//         msg->header.stamp = this->now();
//         msg->header.frame_id = "camera_frame";
//         msg->height = 480;
//         msg->width = 640;
//         msg->encoding = "bgr8";
//         msg->is_bigendian = false;
//         msg->step = msg->width * 3;
        
//         // Копируем данные
//         GstMapInfo map;
//         if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
//             msg->data.resize(map.size);
//             std::copy(map.data, map.data + map.size, msg->data.begin());
//             gst_buffer_unmap(buffer, &map);
            
//             // Публикуем сообщение
//             publisher_->publish(*msg);
//         }
        
//         gst_sample_unref(sample);
//         return GST_FLOW_OK;
//     }
    
//     void print_pipeline_errors()
//     {
//         if (!pipeline_) return;
        
//         GstBus* bus = gst_element_get_bus(pipeline_);
//         GstMessage* msg = gst_bus_poll(bus, GST_MESSAGE_ERROR, 0);
        
//         if (msg) {
//             GError* err = NULL;
//             gchar* debug = NULL;
            
//             gst_message_parse_error(msg, &err, &debug);
//             RCLCPP_ERROR(get_logger(), "Pipeline error: %s", err->message);
//             if (debug) {
//                 RCLCPP_ERROR(get_logger(), "Debug details: %s", debug);
//             }
            
//             g_error_free(err);
//             g_free(debug);
//             gst_message_unref(msg);
//         }
        
//         gst_object_unref(bus);
//     }
    
//     void check_pipeline_state()
//     {
//         if (!pipeline_) return;
        
//         GstState state;
//         gst_element_get_state(pipeline_, &state, NULL, GST_CLOCK_TIME_NONE);
        
//         if (state != GST_STATE_PLAYING) {
//             RCLCPP_WARN(get_logger(), "Pipeline not playing (state: %d), attempting restart...", state);
//             restart_pipeline();
//         }
//     }
    
//     void restart_pipeline()
//     {
//         cleanup_gstreamer();
//         if (!initialize_gstreamer()) {
//             RCLCPP_ERROR(get_logger(), "Failed to restart pipeline");
//         }
//     }
    
//     void cleanup_gstreamer()
//     {
//         if (pipeline_) {
//             gst_element_set_state(pipeline_, GST_STATE_NULL);
//             gst_object_unref(pipeline_);
//             pipeline_ = nullptr;
//         }
//     }
    
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     GstElement* pipeline_;
//     std::string camera_serial_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

// ------------------------------------------------------


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// // #include <sensor_msgs/msg/compressed_image.hpp>
// #include <gst/gst.h>
// #include <gst/app/gstappsink.h>
// #include <cv_bridge/cv_bridge.hpp>
// #include <opencv2/opencv.hpp>
// #include "rclcpp/qos.hpp"

// class CameraPublisher : public rclcpp::Node
// {
// public:
//     CameraPublisher() : Node("camera_publisher")
//     {
//         // Initialize publisher
//         // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

//         rclcpp::QoS qos_profile(10);
//         qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
//         qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
//         qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
//         // publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_compressed", qos_profile);
//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos_profile);

        
//         // Initialize GStreamer
//         gst_init(NULL, NULL);
        
//         // Create pipeline (modified to use appsink instead of ximagesink)
//         pipeline_ = gst_parse_launch(
//             "tcambin name=source ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink", 
//             NULL);
        
//         if (!pipeline_) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline");
//             return;
//         }
        
//         // Get the appsink element
//         GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
//         if (!sink) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to get appsink");
//             return;
//         }
        
//         // Set appsink properties
//         g_object_set(sink, "emit-signals", TRUE, "sync", FALSE, NULL);
//         g_signal_connect(sink, "new-sample", G_CALLBACK(&CameraPublisher::new_sample_callback), this);
//         gst_object_unref(sink);
        
//         // Start pipeline
//         if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
//             return;
//         }
        
//         RCLCPP_INFO(this->get_logger(), "Camera publisher started");
//     }
    
//     ~CameraPublisher()
//     {
//         if (pipeline_) {
//             gst_element_set_state(pipeline_, GST_STATE_NULL);
//             gst_object_unref(pipeline_);
//         }
//     }
    
// private:
//     static GstFlowReturn new_sample_callback(GstElement* sink, CameraPublisher* node)
//     {
//         GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
//         if (!sample) {
//             return GST_FLOW_ERROR;
//         }
        
//         GstBuffer* buffer = gst_sample_get_buffer(sample);
//         GstMapInfo map;
        
//         if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
//             // Create OpenCV Mat from buffer
//             cv::Mat frame(cv::Size(2048, 1536), CV_8UC3, (void*)map.data, cv::Mat::AUTO_STEP);
//             cv::namedWindow("Display window", cv::WINDOW_NORMAL);
//             cv::resizeWindow("Display window", 1024, 768); // Уменьшаем для отображения
//             cv::imshow("Display window", frame);
//             cv::waitKey(1);
//             // // Convert to ROS message
//             auto msg = cv_bridge::CvImage(
//                 std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
//             msg->header.stamp = node->now();
//             msg->header.frame_id = "camera";

//             // std::vector<uchar> compressed_data;
//             // cv::imencode(".jpg", frame, compressed_data);
            
//             // Создание CompressedImage сообщения
//             // auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
//             // msg->header.stamp = node->now();
//             // msg->header.frame_id = "camera";
//             // msg->format = "jpeg";
//             // msg->data = compressed_data;
            
//             // Publish
//             // node->publisher_->publish(std::move(msg));
//             node->publisher_->publish(*msg);
            
//             gst_buffer_unmap(buffer, &map);
//         }
        
//         gst_sample_unref(sample);
//         return GST_FLOW_OK;
//     }
    
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//     GstElement* pipeline_ = nullptr;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

// ---------------------
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
        set_camera_property_double("ExposureTime", 60000.0);
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
            cv::imshow("Camera View", frame);
            cv::waitKey(1);
            
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