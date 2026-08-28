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
    min_range_ = get_parameter("min_range").as_double();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_raw", 10, std::bind(&ServicebotScanFilter::scan_callback, this, _1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan filtered_scan = *msg;

    for (auto &range : filtered_scan.ranges)
    {
      if (range < static_cast<float>(min_range_))
      {
        range = std::numeric_limits<float>::infinity();
      }
    }

    scan_pub_->publish(filtered_scan);
  }

  double min_range_;
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
