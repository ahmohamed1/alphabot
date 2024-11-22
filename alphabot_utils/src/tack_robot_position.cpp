#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

using std::placeholders::_1;

class TrackRobotPosition : public rclcpp::Node{
public:
    TrackRobotPosition() : Node("track_robot_position")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/usb_cam/image", 10, std::bind(&TrackRobotPosition::timer_callback, this, _1)
        );
    }


private:
    void timer_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {   
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg,'BGR8');


    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackRobotPosition>());
  rclcpp::shutdown();
  return 0;
}