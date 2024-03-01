#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "image_tools_sdfr/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "image_tools_sdfr/visibility_control.h"

#include "./policy_maps.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
        image_tools_sdfr::ROSCvMatContainer,
        sensor_msgs::msg::Image
);

namespace image_tools_sdfr {
    class Brightness : public rclcpp::Node {
    public:
        IMAGE_TOOLS_SDFR_PUBLIC
        explicit Brightness(const rclcpp::NodeOptions &options)
                : Node("Brightness", options) {
            setvbuf(stdout, NULL, _IONBF, BUFSIZ);
            // Do not execute if a --help option was provided
            if (help(options.arguments())) {
                // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
                // from a container.
                exit(0);
            }
            parse_parameters();
            initialize();
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub_;

        IMAGE_TOOLS_SDFR_LOCAL
        void initialize() {
            // Set quality of service profile based on command line options.
            auto qos = rclcpp::QoS(
                    rclcpp::QoSInitialization(
                            // The history policy determines how messages are saved until taken by
                            // the reader.
                            // KEEP_ALL saves all messages until they are taken.
                            // KEEP_LAST enforces a limit on the number of messages that are saved,
                            // specified by the "depth" parameter.
                            history_policy_,
                            // Depth represents how many messages to store in history when the
                            // history policy is KEEP_LAST.
                            depth_));
            // The reliability policy can be reliable, meaning that the underlying transport layer will try
            // ensure that every message gets received in order, or best effort, meaning that the transport
            // makes no guarantees about the order or reliability of delivery.
            qos.reliability(reliability_policy_);
            pub_ = this->create_publisher<std_msgs::msg::Bool>("light", 10);
            pos_pub_ = this->create_publisher<geometry_msgs::msg::Point>("light_pos", 10);

            auto callback =
                    [this](const image_tools_sdfr::ROSCvMatContainer &container) {
                        process_image(container, show_image_, this->get_logger());
                    };

            RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
            sub_ = create_subscription<image_tools_sdfr::ROSCvMatContainer>(topic_, qos, callback);

            if (window_name_ == "") {
                // If no custom window name is given, use the topic name
                window_name_ = sub_->get_topic_name();
            }
        }

        IMAGE_TOOLS_SDFR_LOCAL
        bool help(const std::vector <std::string> args) {
            if (std::find(args.begin(), args.end(), "--help") != args.end() ||
                std::find(args.begin(), args.end(), "-h") != args.end()) {
                std::stringstream ss;
                ss << "Usage: Brightness [-h] [--ros-args [-p param:=value] ...]" << std::endl;
                ss << "Subscribe to an image topic and show the images." << std::endl;
                ss << "Example: ros2 run image_tools Brightness --ros-args -p reliability:=best_effort";
                ss << std::endl
                   << std::endl;
                ss << "Options:" << std::endl;
                ss << "  -h, --help\tDisplay this help message and exit";
                ss << std::endl
                   << std::endl;
                ss << "Parameters:" << std::endl;
                ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
                ss << std::endl;
                ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
                ss << std::endl;
                ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
                ss << std::endl;
                ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
                ss << " Default value is 10";
                ss << std::endl;
                ss << "  show_image\tShow the image. Either 'true' (default) or 'false'";
                ss << std::endl;
                ss << "  window_name\tName of the display window. Default value is the topic name";
                ss << std::endl;
                std::cout << ss.str();
                return true;
            }
            return false;
        }

        IMAGE_TOOLS_SDFR_LOCAL
        void parse_parameters() {
            // Parse 'reliability' parameter
            rcl_interfaces::msg::ParameterDescriptor reliability_desc;
            reliability_desc.description = "Reliability QoS setting for the image subscription";
            reliability_desc.additional_constraints = "Must be one of: ";
            for (auto entry: name_to_reliability_policy_map) {
                reliability_desc.additional_constraints += entry.first + " ";
            }
            const std::string reliability_param = this->declare_parameter(
                    "reliability", "reliable", reliability_desc);
            auto reliability = name_to_reliability_policy_map.find(reliability_param);
            if (reliability == name_to_reliability_policy_map.end()) {
                std::ostringstream oss;
                oss << "Invalid QoS reliability setting '" << reliability_param << "'";
                throw std::runtime_error(oss.str());
            }
            reliability_policy_ = reliability->second;

            // Parse 'history' parameter
            rcl_interfaces::msg::ParameterDescriptor history_desc;
            history_desc.description = "History QoS setting for the image subscription";
            history_desc.additional_constraints = "Must be one of: ";
            for (auto entry: name_to_history_policy_map) {
                history_desc.additional_constraints += entry.first + " ";
            }
            const std::string history_param = this->declare_parameter(
                    "history", name_to_history_policy_map.begin()->first, history_desc);
            auto history = name_to_history_policy_map.find(history_param);
            if (history == name_to_history_policy_map.end()) {
                std::ostringstream oss;
                oss << "Invalid QoS history setting '" << history_param << "'";
                throw std::runtime_error(oss.str());
            }
            history_policy_ = history->second;

            // Declare and get remaining parameters
            depth_ = this->declare_parameter("depth", 10);
            show_image_ = this->declare_parameter("show_image", true);
            window_name_ = this->declare_parameter("window_name", "");
        }

        /// Convert the ROS Image message to an OpenCV matrix and display it to the user.
        // \param[in] container The image message to show.
        IMAGE_TOOLS_SDFR_LOCAL
        void process_image(
                const image_tools_sdfr::ROSCvMatContainer &container, bool show_image, rclcpp::Logger logger) {
            RCLCPP_INFO(logger, "Received image #%s", container.header().frame_id.c_str());
            std::cerr << "Received image #" << container.header().frame_id.c_str() << std::endl;

            cv::Mat frame = container.cv_mat();
            cv::Mat grayscale_frame;
            if (show_image) {
                cv::Mat frame = container.cv_mat();

                if (frame.type() == CV_8UC3 /* rgb8 */) {
                    cv::cvtColor(frame, grayscale_frame, cv::COLOR_RGB2GRAY);
                } else if (frame.type() == CV_8UC2) {
                    cv::Mat temp_frame;
                    container.is_bigendian() ? cv::cvtColor(frame, temp_frame, cv::COLOR_YUV2GRAY_UYVY) : cv::cvtColor(
                            frame, temp_frame, cv::COLOR_YUV2GRAY_YUYV);
                    grayscale_frame = temp_frame;
                } else {
                    RCLCPP_ERROR(logger, "Unsupported image format for brightness calculation");
                    return;
                }

                double average_brightness = cv::mean(grayscale_frame)[0];
                bool light = average_brightness > 50.0; // abra cabra

                // Log the average brightness
                RCLCPP_INFO(logger, "Average brightness: %f", average_brightness);
                // Log if the brightness is abpve or below a threshold
                RCLCPP_INFO(logger, "Light: %s", light ? "true" : "false");

                auto message = std_msgs::msg::Bool();
                message.data = light;
                pub_->publish(message);

                // Apply Threshold for Light Detection
                int threshold_value = 250; // Adjust this as needed
                cv::Mat thresholded_frame;
                cv::threshold(grayscale_frame, thresholded_frame, threshold_value, 255, cv::THRESH_BINARY);

                // Find Light Position (COG)
                cv::Moments moments = cv::moments(thresholded_frame);
                double m10 = moments.m10;
                double m01 = moments.m01;
                double area = moments.m00;

                int posX = -1;
                int posY = -1;
                if (area > 1000.0) // Filter based on minimum light area
                {
                    posX = m10 / area;
                    posY = m01 / area;


                    cv::Mat rgb_frame;
                    cv::cvtColor(thresholded_frame, rgb_frame, cv::COLOR_GRAY2BGR);
                    cv::circle(rgb_frame, cv::Point(posX, posY), 5, cv::Scalar(255, 0, 255), cv::FILLED, 8, 0);

                    // Show the image in a window
                    // cv::imshow(window_name_, grayscale_frame);
                    cv::imshow(window_name_, rgb_frame);
                    // Draw the screen and wait for 1 millisecond.
                    
                    //int image_width = container.cv_mat().cols;
                    //int image_height = container.cv_mat().rows;
                    // Scale the position to the range [-1, 1] for easy use fro control
                    //double scaled_posX = ((double)posX / (double)image_width) * 2.0 - 1.0;
                    //double scaled_posY = ((double)posY / (double)image_height) * 2.0 - 1.0;
                    geometry_msgs::msg::Point point;
                    point.x = posX;
                    point.y = posY;
                    point.z = 0;

                    pos_pub_->publish(point);
                    cv::waitKey(1);
                }
            }
        }

        rclcpp::Subscription<image_tools_sdfr::ROSCvMatContainer>::SharedPtr sub_;
        size_t depth_ = rmw_qos_profile_default.depth;
        rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
        rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
        bool show_image_ = true;
        std::string topic_ = "image";
        std::string window_name_;
    };
} // namespace image_tools_sdfr

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools_sdfr::Brightness)
