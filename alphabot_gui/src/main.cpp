#include <QApplication>
#include "alphabot_gui/mainWindow.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // Initialize both ROS2 and Qt
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("alphabot_gui_node");

    MainWindow w(node);
    w.show();

    int ret = app.exec();

    rclcpp::shutdown();
    return ret;
}
