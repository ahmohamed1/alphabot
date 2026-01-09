#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class FollowGap : public rclcpp::Node
{
public:

FollowGap():Node("follow_gap")
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
        10, std::bind(&FollowGap::scan_callback, this, _1));
    
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

}


private:

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;


double kp = 2;
double angle = 45;
double desire_distance = 0.6;
double velocity = 0.2;
std::vector<double> processed_scan;

void process_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    this->processed_scan.clear();
    for (int i = 0; i < (int)scan_msg->ranges.size(); i++)
    {   
        if(scan_msg->ranges[i] > 3.0)
            this->processed_scan.push_back(0);
        else{
            this->processed_scan.push_back(scan_msg->ranges[i]);
            }
    }
    
}


std::pair<int, int> find_max_gap() {
    // Return the start index & end index of the max gap in free_space_ranges
    int largest_starting_i = 0;
    int longest_gap = 0;
    int current_gap = 0;
    for (size_t i = 0; i < (size_t)this->processed_scan.size(); i++) {
        if (this->processed_scan[i] < 0.5) {
            current_gap = 0;
        } else {
            current_gap++;
            if (current_gap > longest_gap) {  // Update the largest gap
                largest_starting_i = i - current_gap;
                longest_gap = current_gap;
            }
        }
    }
    return std::make_pair(largest_starting_i, longest_gap);
}

int find_best_point(int starting_i, int gap_distance) {
    // Start_i & end_i are start and end indicies of max-gap range, respectively
    // Return index of best point in ranges
    // Naive: Choose the furthest point within ranges and go there
    int farthest_i = starting_i;
    double farthest_distance = 0;
    for (int i = starting_i; i < starting_i + gap_distance; i++) {
        if (this->processed_scan[i] > farthest_distance) {
            farthest_i = i;
            farthest_distance = this->processed_scan[i];
        }
    }
    return farthest_i;
}


void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    
    // Find max length gap
    std::pair<int, int> p;
    process_scan(scan_msg);
    p = find_max_gap();

    // Find the best point in the gap
    double best_angle_i = find_best_point(p.first, p.second);

    // Compute the turing speed for diff
    auto angle_cmd = geometry_msgs::msg::Twist();
    double turning = 2 * scan_msg->angle_min + best_angle_i * scan_msg->angle_increment;
    RCLCPP_INFO(this->get_logger(), "a: '%f'", turning);  // Output to log;
    angle_cmd.angular.z = turning;
    angle_cmd.linear.x = velocity;
    cmd_vel_pub_->publish(angle_cmd);

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
    rclcpp::spin(std::make_shared<FollowGap>());
    rclcpp::shutdown();
    return 0;
}