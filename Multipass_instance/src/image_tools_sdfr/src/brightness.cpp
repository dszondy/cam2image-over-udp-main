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
#include <vector>

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
            pub_ = this->create_publisher<std_msgs::msg::Bool>("light", 10);
            pos_pub_ = this->create_publisher<geometry_msgs::msg::Point>("light_pos", 10);

            auto callback =
                    [this](const image_tools_sdfr::ROSCvMatContainer &container) {
                        process_image(container, show_image_, this->get_logger());
                    };

            RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
            sub_ = create_subscription<image_tools_sdfr::ROSCvMatContainer>(topic_, 10, callback);

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
                ss << "Displays the birghtness and calculates the brightest spots location." << std::endl;
                ss << "Example: ros2 run image_tools Brightness --ros-args -p threashold:=250";
                ss << std::endl
                   << std::endl;
                ss << "Options:" << std::endl;
                ss << "  -h, --help\tDisplay this help message and exit";
                ss << std::endl
                   << std::endl;
                ss << "Parameters:" << std::endl;
                ss << "  threashold\tThe threashold value for the brightness calculation";
                ss << std::endl;
                std::cout << ss.str();
                return true;
            }
            return false;
        }

        IMAGE_TOOLS_SDFR_LOCAL
        void parse_parameters() {
            // Declare and get remaining parameters
            threashold_ = this->declare_parameter("threashold", 10);
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
                bool light = average_brightness > threashold_; // abra cabra

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

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(thresholded_frame, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                if(contours.size() > 0)
                {
                    auto c = max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return a.size() < b.size();
                    });
                    auto mu = cv::moments(*c, false);
                    auto mc = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);

                    cv::Mat rgb_frame;
                    cv::cvtColor(thresholded_frame, rgb_frame, cv::COLOR_GRAY2BGR);
                    cv::circle(rgb_frame, mc, 5, cv::Scalar(255, 0, 255), cv::FILLED, 8, 0);

                    cv::imshow(window_name_, rgb_frame);
                    geometry_msgs::msg::Point point;

                    point.x = mc.x;
                    point.y = mc.y;
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
        int threashold_ = 250;
    };
} // namespace image_tools_sdfr

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools_sdfr::Brightness)
