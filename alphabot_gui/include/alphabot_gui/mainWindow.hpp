#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "ui_alphabot_gui.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <QGraphicsScene>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <fstream>
#include <mutex>

#include <QListWidget>
#include <QVector>
#include <QString>


#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>


struct Location {
    QString name;
    double x;
    double y;
    double theta;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void emergencyPressed();
    void manualController(const QString &command);
    void changeSpeed(int value);
    void periodicBatteryUpdate();

    void addLocation();
    void editLocation();
    void startNavigation();

    // void updateFrame();                      // Slot to update the reference frame
    // void updateMapReceivedIndicator(bool received);  // Updates the map received indicator in the GUI


private:
    Ui::MainWindow ui;
    std::shared_ptr<rclcpp::Node> node_;
    QTimer *battery_timer;

    // ROS publisher for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    geometry_msgs::msg::Pose current_pose_;
    std::mutex pose_mutex_;

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    double linear_speed_ = 0.1;   // speed from slider
    double angular_speed_ = 0.4;  // fixed for turns

    void loadLocations();
    void saveLocations();
    void updateLocationList();
    void saveLocationsToFile();
    bool editLocationDialog(Location &loc);

    QVector<Location> locations;
    QString locationsFile = "locations.txt";   // file storage path


    // Rviz 

    void initializeRViz();                   // Initializes RViz components
    //void DisplayGrid();                      // Sets up the grid and TF displays
    //void setupRobotModel();                  // Sets up the robot model display
    void setupJoystickControls();            // Initializes joystick buttons for movement control
    //void setupMapSubscriber();               // Sets up the map subscriber to listen for map data
    
    
    
    void setupGridDisplay();
    void setupTFDisplay();
    void setupMapDisplay();
    void setupRobotModelDisplay();
    void setupMapSubscriber();
    void setupLaserScanDisplay();


    rviz_common::Display *grid_;             // Grid display object
    rviz_common::Display *tf_display_;       // TF display object
    rviz_common::Display *map_display_;      // Map display object
    rviz_common::Display *robot_model_display_; // RobotModel display object
    rviz_common::RenderPanel* renderPanel_ = nullptr;
    rviz_common::VisualizationManager* manager_ = nullptr;

    // ROS node and publisher for /cmd_vel
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_ = nullptr;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber_; // Subscriber for map data
    bool mapReceived_;                       // Boolean flag to track map data reception

};

#endif // MAINWINDOW_HPP
