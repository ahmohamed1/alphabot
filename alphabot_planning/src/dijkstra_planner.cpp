#include "alphabot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace alphabot_planning
{
DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, 
        std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",
        10, std::bind(&DijkstraPlanner::goalCallback, this, std::placeholders::_1));
    
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/dijkstra/path",10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/dijkstra/visited_map",10);
}

void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    map_ = map;
    visited_map_.header.frame_id = map->header.frame_id;
    visited_map_.info = map->info;
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);
}
void DijkstraPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    if(!map_){
        RCLCPP_ERROR(get_logger(), "No map recived!");
        return;
    }

    visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);

    geometry_msgs::msg::TransformStamped map_to_base_tf;
    try{
        map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footpring", tf2::TimePointZero);
    }catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Could not transform from the map to base_footprint");
        return;
    }

    geometry_msgs::msg::Pose map_to_base_pose;
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

    auto path = plan(map_to_base_pose, pose->pose);
    if(path.poses.empty()){
        RCLCPP_INFO(get_logger(), "Shortest path found");
        path_pub_->publish(path);
    }else{
        RCLCPP_WARN(get_logger(), "No path found to the goal!");
    }
}

nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
{

}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<alphabot_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}