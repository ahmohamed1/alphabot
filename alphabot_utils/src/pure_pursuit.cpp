#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


double to_radians(double theta) {
    return M_PI * theta / 180.0;

}

double to_degrees(double theta) {
    return theta * 180.0 / M_PI;
}



class PursuitNode : public rclcpp::Node
{
public:
    PursuitNode() : Node("pursuit_node")
    {
        // Subscriber to the path to follow
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PursuitNode::pathCallback, this, std::placeholders::_1));
        // Subscriber to the current location (odometry)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/alphabot_controller/odom", 10, std::bind(&PursuitNode::odomCallback, this, std::placeholders::_1));
        // Publisher for the command (e.g., velocity)
        command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 10);

        // Set the lookahead distance
        lookahead_distance_ = this->declare_parameter("lookahead_distance", 0.4);

        RCLCPP_INFO(this->get_logger(), "Pursuit node has been started.");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Store the path
        path_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Path has been recived");
        // publish_marker_lookahead(path_->poses[index_main_loop].pose.position.x, path_->poses[index_main_loop].pose.position.y);        

        // index_main_loop += 1;
        // if(path_->poses.size() <= index_main_loop)
        //     index_main_loop = 0;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the orientation quaternion from the message
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // Convert quaternion to Euler angles
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // Get the current position from odometry
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
        current_heading_ = yaw;
        // RCLCPP_INFO(this->get_logger(), "Position recived");
        // Call the pursuit algorithm
        pursuePath();
    }

    void pursuePath()
    {
        if (!path_ || path_->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty or not received yet.");
            return;
        }
        
        geometry_msgs::msg::Twist command;
         // Find the nearest point to the lookahead distance
        auto target_pose = findLookaheadPoint();
        publish_marker_lookahead(target_pose.position.x, target_pose.position.y);        
        double alpha = std::atan2((target_pose.position.y - current_position_y_), (target_pose.position.x - current_position_x_)) - current_heading_;
        double delta = std::atan2(2.0 * WB * std::sin(alpha) / Lf, 1.0);
        command.linear.x = get_velocity(delta);
        command.angular.z = delta;
        // RCLCPP_INFO(this->get_logger(), "Turing angle : %f", delta);
        // Publish the command
        command_pub_->publish(command);

    }


    void publish_marker_lookahead(double _x, double _y){
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 1.0;

        marker.pose.position.x = _x;
        marker.pose.position.y = _y;
        marker.pose.position.z = 0.0;
        marker.id = 1;
        marker_pub_->publish(marker);
        // RCLCPP_WARN(this->get_logger(), "Marker published");
    }


    geometry_msgs::msg::Pose findLookaheadPoint()
    {
        double dx, dy, distance;
        geometry_msgs::msg::Pose pose_;

        if (visited_index == -1)
        {
            for (size_t i = 0; i < path_->poses.size(); i++)
            {
                dx = path_->poses[i].pose.position.x - current_position_x_;
                dy = path_->poses[i].pose.position.y - current_position_y_;
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance >= lookahead_distance_)
                {
                    visited_index = i;
                    return path_->poses[i].pose;
                }
            }
        }
        else
        {
            for (size_t i = visited_index; i < path_->poses.size(); i++)
            {
                dx = path_->poses[i].pose.position.x - current_position_x_;
                dy = path_->poses[i].pose.position.y - current_position_y_;
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance >= lookahead_distance_)
                {
                    visited_index = i;
                    return path_->poses[i].pose;
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "No valid lookahead point found within the lookahead distance.");
        return geometry_msgs::msg::Pose(); // Returning a default pose
    }

    double get_velocity(double steering_angle) {
        double velocity = 0;

        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 1.0;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 0.5;
        } else {
            velocity = 0.2;
        }
        return velocity;
    }

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Current position of the pursuer
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_heading_ = 0.0;
    double lookahead_distance_ = 0.8;
    double Lf = 0.5;
    double WB = 0.36;

    // this to store the index we visited before
    int visited_index = -1;
    int index_main_loop = 0;
    // Stored path
    nav_msgs::msg::Path::SharedPtr path_;

    // Gain for the proportional controller
    double kp_ = 0.8;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PursuitNode>());
    rclcpp::shutdown();
    return 0;
}
