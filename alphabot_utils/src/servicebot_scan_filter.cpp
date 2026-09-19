#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

// Servicebot's laser is mounted at the center of the robot, so the small
// side walls next to the sensor housing show up as very close-range
// "obstacles" on the left/right of the scan. This node discards any range
// reading closer than min_range (treating it as no-return / infinity), so
// costmap/AMCL/SLAM no longer see the robot's own housing as an obstacle.
class ServicebotScanFilter : public rclcpp::Node
{
public:
  ServicebotScanFilter() : Node("servicebot_scan_filter")
  {
    declare_parameter<double>("min_range", 0.25);
    declare_parameter<double>("wall_max_range", 0.40);
    declare_parameter<double>("left_wall_min_angle", 1.31);
    declare_parameter<double>("left_wall_max_angle", 1.83);
    declare_parameter<double>("right_wall_min_angle", -1.83);
    declare_parameter<double>("right_wall_max_angle", -1.31);
    min_range_ = get_parameter("min_range").as_double();
    wall_max_range_ = get_parameter("wall_max_range").as_double();
    left_wall_min_angle_ = get_parameter("left_wall_min_angle").as_double();
    left_wall_max_angle_ = get_parameter("left_wall_max_angle").as_double();
    right_wall_min_angle_ = get_parameter("right_wall_min_angle").as_double();
    right_wall_max_angle_ = get_parameter("right_wall_max_angle").as_double();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_raw", 10, std::bind(&ServicebotScanFilter::scan_callback, this, _1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan filtered_scan = *msg;

    for (std::size_t index = 0; index < filtered_scan.ranges.size(); ++index)
    {
      auto &range = filtered_scan.ranges[index];
      const double angle = filtered_scan.angle_min +
        static_cast<double>(index) * filtered_scan.angle_increment;
      const bool is_wall_sector =
        (angle >= left_wall_min_angle_ && angle <= left_wall_max_angle_) ||
        (angle >= right_wall_min_angle_ && angle <= right_wall_max_angle_);

      if (range < static_cast<float>(min_range_))
      {
        range = std::numeric_limits<float>::infinity();
      }
      else if (is_wall_sector && range <= static_cast<float>(wall_max_range_))
      {
        range = std::numeric_limits<float>::infinity();
      }
    }

    scan_pub_->publish(filtered_scan);
  }

  double min_range_;
  double wall_max_range_;
  double left_wall_min_angle_;
  double left_wall_max_angle_;
  double right_wall_min_angle_;
  double right_wall_max_angle_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicebotScanFilter>());
  rclcpp::shutdown();
  return 0;
}
