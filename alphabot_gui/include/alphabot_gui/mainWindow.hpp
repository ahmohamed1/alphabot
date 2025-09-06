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

};

#endif // MAINWINDOW_HPP
