#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"


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
        // Set the lookahead distance
        lookahead_distance_ = this->declare_parameter("lookahead_distance", 0.6);

        RCLCPP_INFO(this->get_logger(), "Pursuit node has been started.");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Store the path
        path_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Path has been recived");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the current position from odometry
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
        current_heading_ = msg->pose.pose.position.y;
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

         // Find the nearest point to the lookahead distance
        auto target_pose = findLookaheadPoint();


        geometry_msgs::msg::Twist command;
        double alpha = std::atan2((target_pose.position.y - current_position_y_), (target_pose.position.x - current_position_x_));
        double delta = std::atan2(2.0 * WB * std::sin(alpha) / Lf, 1.0);
        command.linear.x = get_velocity(delta);
        command.angular.z = delta;
        // Publish the command
        command_pub_->publish(command);
    }

    geometry_msgs::msg::Pose find_lookahead_point() {
        for (const auto& point : path_->poses) {
            double dx = point.pose.position.x - current_position_x_;
            double dy = point.pose.position.y - current_position_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance >= lookahead_distance_) {
                std::cout << "Lookahead point found: x=" << point.pose.position.x << ", y=" << point.pose.position.y << std::endl;
                return point.pose;
            }
        }
        RCLCPP_WARN(this->get_logger(), "No lookahead point found within lookahead distance.");
        geometry_msgs::msg::Pose default_pose;
        default_pose.position.x = -1.0;
        default_pose.position.y = -1.0;
        return default_pose; // or any other suitable default value
    }

    geometry_msgs::msg::Pose findLookaheadPoint()
    {
        double dx;
        double dy;
        double distance;
        geometry_msgs::msg::Pose pose_;
        if (visited_index == NULL)
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


    double get_velocity(double steering_angle) {
        double velocity = 0;

        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 0.5;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 0.25;
        } else {
            velocity = 0.1;
        }
        return velocity;
    }

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;

    // Current position of the pursuer
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_heading_ = 0.0;
    double lookahead_distance_ = 0.8;
    double Lf = 0.5;
    double WB = 0.36;

    // this to store the index we visited before
    int visited_index = NULL;
    // Stored path
    nav_msgs::msg::Path::SharedPtr path_;

    // Gain for the proportional controller
    double kp_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PursuitNode>());
    rclcpp::shutdown();
    return 0;
}
