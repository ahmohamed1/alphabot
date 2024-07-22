#include "alphabot_mapping/mapping_with_known_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include"tf2/utils.h"


using namespace std::placeholders;

namespace alphabot_mapping
{

unsigned int pose_to_cell(const Pose & pose, const nav_msgs::msg::MapMetaData & map_info)
{
    return map_info.width * pose.y + pose.x;
}

Pose coordinate_to_pose(const double px, const double py, const nav_msgs::msg::MapMetaData & map_info)
{
    Pose pose;
    pose.x = std::round((px - map_info.origin.position.x) / map_info.resolution);
    pose.y = std::round((py - map_info.origin.position.y) / map_info.resolution);
    return pose;
}

bool pose_on_map(const Pose & pose, const nav_msgs::msg::MapMetaData & map_info)
{
    return pose.x < static_cast<int>(map_info.width)  && pose.x >= 0 &&
           pose.y < static_cast<int>(map_info.height) && pose.y >= 0;
}

MappingWihtKnownPoses::MappingWihtKnownPoses(const std::string & name) : Node(name)
{
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    map_.info.resolution = get_parameter("resolution").as_double();
    map_.info.width = std::round(width/map_.info.resolution);   // the size of the map in pixel
    map_.info.height = std::round(height/map_.info.resolution); // the size of the map in pixel
    map_.info.origin.position.x = - std::round(width / 2.0);
    map_.info.origin.position.y = - std::round(height / 2.0);
    map_.header.frame_id = "odom";
    map_.data = std::vector<int8_t>(map_.info.height * map_.info.width, -1);


    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",10, 
                std::bind(&MappingWihtKnownPoses::scan_callback, this, _1));

    time_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MappingWihtKnownPoses::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void MappingWihtKnownPoses::scan_callback(const sensor_msgs::msg::LaserScan & scan)
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(map_.header.frame_id,scan.header.frame_id, tf2::TimePointZero);
    }
    catch(const tf2::TransformException& e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to transform between odom and /base_footprint");
        return;
    }
    
    Pose robot_pos = coordinate_to_pose(t.transform.translation.x, t.transform.translation.y, map_.info);
    if(!pose_on_map(robot_pos, map_.info))
    {
        RCLCPP_ERROR(get_logger(), "[INFO] The rpbpt is out of the Map!");
        return;
    }

    unsigned int robot_cell = pose_to_cell(robot_pos, map_.info);
    map_.data.at(robot_cell) = 100; // color the cell
}


void MappingWihtKnownPoses::timer_callback()
{
    // Convert to occupancy grid and publish
    // std::transform(probability_map_.begin(), probability_map_.end(), map_.data.begin(), [](double value){
    //     return logodds2prob(value) * 100;
    // });
    map_.header.stamp = get_clock()->now();
    map_pub_->publish(map_);
}

} // End of namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<alphabot_mapping::MappingWihtKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}