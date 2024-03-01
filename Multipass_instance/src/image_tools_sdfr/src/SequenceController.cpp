#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <vector>
#include <math.h>

namespace image_tools_sdfr
{
    // Define a custom ROS node class
    class SequenceControllerNode : public rclcpp::Node
    {
    public:
        SequenceControllerNode(const rclcpp::NodeOptions & options) : Node("sequence_controller", options)
        {
            // Parameters for square trajectory
            side_length_ = 1.0;   // Meters
            speed_ = 1;         // Meters per second
            turn_duration_ = 1.0; // Seconds for each 90-degree turn

            // Simple waypoints for a square
            waypoints_.push_back(geometry_msgs::msg::TwistStamped()); // Default initialization
            waypoints_.back().twist.linear.x = speed_;               // GO
            waypoints_.push_back(geometry_msgs::msg::TwistStamped());
            waypoints_.back().twist.linear.x = 0;               // Stop


            // Publisher for velocity commands
            publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/input/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "Publishing waypoint command to /input/cmd_vel");

            // Use a timer for publishing at a regular rat
            //double move_duration = distance / speed_;  // In secondse
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(5000), // 5 sec --> 5 sec* 5m/s = 5m
                std::bind(&SequenceControllerNode::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Publishing waypoint command");

            // Send the current waypoint command
            publisher_->publish(waypoints_[waypoint_index_]);

            // Update waypoint index for the next command
            waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();

            // Check if we've completed all waypoints (adjust logic if needed)
            if (waypoint_index_ == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Trajectory completed!");
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
        std::vector<geometry_msgs::msg::TwistStamped> waypoints_;
        size_t waypoint_index_ = 0;
        double side_length_;
        double speed_;
        double turn_duration_;
    };

} // namespace image_tools_sdfr
RCLCPP_COMPONENTS_REGISTER_NODE(image_tools_sdfr::SequenceControllerNode);
