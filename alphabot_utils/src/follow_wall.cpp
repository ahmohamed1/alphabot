#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class FollowWall : public rclcpp::Node
{
public:

FollowWall():Node("follow_wall")
{
    declare_parameter<double>("kp", 1.0);
    declare_parameter<double>("angle", 45.0);
    declare_parameter<double>("desire_distance", 1);
    declare_parameter<double>("velocity", 0.2);
    kp = get_parameter("kp").as_double();
    angle = get_parameter("angle").as_double();
    desire_distance = get_parameter("desire_distance").as_double();
    velocity = get_parameter("velocity").as_double();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",
        10, std::bind(&FollowWall::scan_callback, this, _1));
    
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

}


private:

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;


double kp = 2;
double angle = 45;
double desire_distance = 0.6;
double velocity = 0.2;

void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    // desired angle is Kp(-y-L*sin(theta))
    double a = get_range(scan, to_radians(angle));
    double b = get_range(scan, to_radians(0));
    double theta = to_radians(45);
    double alpha = std::atan((a*std::cos(theta) - b)/(a*std::sin(theta)));
    double AB = b * std::cos(alpha);

    // Compute the turing speed for diff
    double error = AB - desire_distance;
    double computed_angle = kp * error;
    auto angle_cmd = geometry_msgs::msg::Twist();
    angle_cmd.angular.z = computed_angle;
    angle_cmd.linear.x = velocity;
    cmd_vel_pub_->publish(angle_cmd);

}

double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
{
    assert(angle >= scan_msg->angle_min && angle <= scan_msg->angle_max); // Angle must be within range
    int i = (angle - scan_msg->angle_min) / (scan_msg->angle_increment);
    if(std::isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max)
    {
        return scan_msg->range_max;
    }
    return scan_msg->ranges[i];

}   

double to_radians(double theta) {
    return M_PI * theta / 180.0;

}

double to_degrees(double theta) {
    return theta * 180.0 / M_PI;
}

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<FollowWall>());
    rclcpp::shutdown();
    return 0;
}