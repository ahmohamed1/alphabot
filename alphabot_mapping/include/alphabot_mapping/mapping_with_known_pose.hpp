#ifndef MAPPING_WITH_KNOWN_POSE_HPP
#define MAPPING_WITH_KNOWN_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace alphabot_mapping
{
    inline const double PRIOR_PROB = 0.5;
    inline const double OCC_PROB = 0.9;
    inline const double FREE_PROB = 0.35;
    
    struct Pose
    {
        Pose() = default;
        Pose(const int px, const int py) : x(px), y(py){}

        int x;
        int y;
    };
    
    unsigned int pose_to_cell(const Pose & pose, const nav_msgs::msg::MapMetaData & map_info);
    Pose coordinate_to_pose(const double px, const double py, const nav_msgs::msg::MapMetaData & map_info);
    bool pose_on_map(const Pose & pose, const nav_msgs::msg::MapMetaData & map_info);

    std::vector<Pose> bresenham(const Pose & start, const Pose & end);
    std::vector<std::pair<Pose, double>> inverse_sensor_model(const Pose & robot_pose, const Pose & beam_pose);

    double prob2logodds(double p);
    double logodd2prob(double l);

class MappingWihtKnownPoses : public rclcpp::Node
{
public:
    MappingWihtKnownPoses(const std::string & name);


private:
    void scan_callback(const sensor_msgs::msg::LaserScan & scan);

    void timer_callback();

    nav_msgs::msg::OccupancyGrid map_;
    std::vector<double> probability_map_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr time_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

};

} // end of namespae
#endif