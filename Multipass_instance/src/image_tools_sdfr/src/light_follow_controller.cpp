#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_tools_sdfr/visibility_control.h"
#include "./policy_maps.hpp"

#include <chrono>
#include <vector>
#include <math.h>

namespace image_tools_sdfr
{
    // Define a custom ROS node class
    class LightFollowControllerNode : public rclcpp::Node
    {
    public:
        IMAGE_TOOLS_SDFR_PUBLIC
        explicit LightFollowControllerNode(const rclcpp::NodeOptions &options) : Node("light_follow_controller", options)
        {
            initialize();
        }

    private:
        IMAGE_TOOLS_SDFR_LOCAL
        void initialize()
        {
            // log
            RCLCPP_INFO(this->get_logger(), "LightFollowControllerNode constructor");

            // Publisher for velocity commands
            publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/input/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "Publishing waypoint command to /input/cmd_vel");
            sub_ = this->create_subscription<geometry_msgs::msg::Point>("light_pos", 10,
                                                                        [this](const geometry_msgs::msg::Point::SharedPtr msg)
                                                                        {
                                                                            this->callback(*msg);
                                                                        });
            RCLCPP_INFO(this->get_logger(), "Subscibed to light_pos");
        }

        void callback(const geometry_msgs::msg::Point msg)
        {
            // log
            RCLCPP_INFO(this->get_logger(), "Received light position: x=%f, y=%f", msg.x, msg.y);
            // Create a new waypoint command
            geometry_msgs::msg::TwistStamped waypoint;
            double speed = 0.2;
            waypoint.twist.linear.x = speed;
            // calculate the angular velocity in radians based on message x that is in range [-1, 1]
            waypoint.twist.angular.z = msg.x * M_PI_4;
            // log
            RCLCPP_INFO(this->get_logger(), "Angular velocity: %f", waypoint.twist.angular.z);

            publisher_->publish(waypoint);
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    };
} // namespace image_tools_sdfr
RCLCPP_COMPONENTS_REGISTER_NODE(image_tools_sdfr::LightFollowControllerNode);
