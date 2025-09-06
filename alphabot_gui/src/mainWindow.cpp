#include "alphabot_gui/mainWindow.hpp"
#include <QInputDialog>
#include <QMessageBox>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <cmath>
#include <mutex>

#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>


MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QMainWindow(parent), node_(node)
{
    ui.setupUi(this);
    initializeRViz();
    // ---------------- ROS Setup ----------------
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&MainWindow::poseCallback, this, std::placeholders::_1)
    );

    // ---------------- GUI Connections ----------------
    connect(ui.pbEmergencyStop, &QPushButton::pressed, this, &MainWindow::emergencyPressed);
    connect(ui.PBForward,  &QPushButton::pressed, [this]() { manualController("FORWARD"); });
    connect(ui.PBBackWord, &QPushButton::pressed, [this]() { manualController("BACKWARD"); });
    connect(ui.PBRight,    &QPushButton::pressed, [this]() { manualController("RIGHT"); });
    connect(ui.PBLeft,     &QPushButton::pressed, [this]() { manualController("LEFT"); });
    connect(ui.PBStop,     &QPushButton::pressed, [this]() { manualController("STOP"); });
    connect(ui.HSSpeed, QOverload<int>::of(&QSlider::valueChanged), this, &MainWindow::changeSpeed);

    // Battery monitor
    battery_timer = new QTimer(this);
    connect(battery_timer, &QTimer::timeout, this, &MainWindow::periodicBatteryUpdate);
    battery_timer->start(3000);

    // Location management
    connect(ui.PBAddNewLocation, &QPushButton::pressed, this, &MainWindow::addLocation);
    connect(ui.PBEditLocation, &QPushButton::pressed, this, &MainWindow::editLocation);
    connect(ui.PBStartNavigation, &QPushButton::pressed, this, &MainWindow::startNavigation);

    // Load saved locations
    loadLocations();
    updateLocationList();


    // Initialize RVIZ setup
    // Create renderPanel_ first
    // renderPanel_ = new rviz_common::RenderPanel();
    
    // QVBoxLayout* layout = new QVBoxLayout(ui.rvizContainer);
    // layout->setContentsMargins(0, 0, 0, 0); // optional, remove spacing
    // layout->addWidget(renderPanel_);

     // Update the fixed frame in VisualizationManager
    // manager_->getRootDisplayGroup()->setFixedFrame(frame_id); // Set for root display group
    // manager_->setFixedFrame(frame_id); // Set for frame manager

    // Call updateFixedFrame() to apply changes across the visualization manager
    //manager_->updateFixedFrame();

//     QTimer::singleShot(100, this, [this]() {
//     setupGridDisplay();
//     setupTFDisplay();
//     setupMapDisplay();
//     setupRobotModelDisplay();
//     setupLaserScanDisplay();
// });


    RCLCPP_INFO(node_->get_logger(), "MainWindow initialized!");
}

MainWindow::~MainWindow()
{
    RCLCPP_INFO(node_->get_logger(), "MainWindow closed.");
}

void MainWindow::emergencyPressed()
{
    RCLCPP_WARN(node_->get_logger(), "Emergency Stop Pressed!");
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
}

void MainWindow::manualController(const QString &command)
{
    geometry_msgs::msg::Twist twist_msg;

    if (command == "FORWARD") twist_msg.linear.x = linear_speed_;
    else if (command == "BACKWARD") twist_msg.linear.x = -linear_speed_;
    else if (command == "LEFT") twist_msg.angular.z = angular_speed_;
    else if (command == "RIGHT") twist_msg.angular.z = -angular_speed_;
    else if (command == "STOP") twist_msg.linear.x = 0.0; twist_msg.angular.z = 0.0;

    cmd_vel_pub_->publish(twist_msg);

    RCLCPP_INFO(node_->get_logger(), "Command: %s, linear: %.2f, angular: %.2f",
                command.toStdString().c_str(), twist_msg.linear.x, twist_msg.angular.z);
}

void MainWindow::changeSpeed(int value)
{
    linear_speed_ = static_cast<double>(value) / 100.0;
    ui.LSpeed->setText(QString::number(linear_speed_, 'f', 2));
    RCLCPP_INFO(node_->get_logger(), "Linear speed set to %.2f m/s", linear_speed_);
}

void MainWindow::periodicBatteryUpdate()
{
    RCLCPP_INFO(node_->get_logger(), "Battery check triggered");
}

void MainWindow::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = msg->pose.pose;
}

void MainWindow::loadLocations()
{
    locations.clear();
    QFile file(locationsFile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QStringList parts = in.readLine().split(",");
        if (parts.size() == 4) {
            Location loc{parts[0], parts[1].toDouble(), parts[2].toDouble(), parts[3].toDouble()};
            locations.append(loc);
        }
    }
    file.close();
}

void MainWindow::saveLocations()
{
    QFile file(locationsFile);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);
    for (const auto &loc : locations)
        out << loc.name << "," << loc.x << "," << loc.y << "," << loc.theta << "\n";
    file.close();
}

void MainWindow::updateLocationList()
{
    ui.LWSaveLocations->clear();
    for (const auto &loc : locations) ui.LWSaveLocations->addItem(loc.name);
}

void MainWindow::addLocation()
{
    bool ok;
    QString name = QInputDialog::getText(this, "Add Location", "Enter location name:", QLineEdit::Normal, "", &ok);
    if (!ok || name.isEmpty()) return;

    geometry_msgs::msg::Pose pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose = current_pose_;
    }

    double qx = pose.orientation.x, qy = pose.orientation.y, qz = pose.orientation.z, qw = pose.orientation.w;
    double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

    Location newLoc{name, pose.position.x, pose.position.y, yaw};
    locations.append(newLoc);
    saveLocations();
    updateLocationList();
}

bool MainWindow::editLocationDialog(Location &loc)
{
    QDialog dlg(this);
    dlg.setWindowTitle("Edit Location");
    auto *form = new QFormLayout(&dlg);
    auto *nameEdit = new QLineEdit(loc.name, &dlg);
    auto *xSpin = new QDoubleSpinBox(&dlg); xSpin->setRange(-1e6, 1e6); xSpin->setDecimals(3); xSpin->setValue(loc.x);
    auto *ySpin = new QDoubleSpinBox(&dlg); ySpin->setRange(-1e6, 1e6); ySpin->setDecimals(3); ySpin->setValue(loc.y);

    form->addRow("Name:", nameEdit);
    form->addRow("X:", xSpin);
    form->addRow("Y:", ySpin);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        loc.name = nameEdit->text();
        loc.x = xSpin->value();
        loc.y = ySpin->value();
        return true;
    }
    return false;
}

void MainWindow::editLocation()
{
    int row = ui.LWSaveLocations->currentRow();
    if (row < 0 || row >= locations.size()) return;

    Location edited = locations[row];
    if (editLocationDialog(edited)) {
        locations[row] = edited;
        saveLocations();
        updateLocationList();
        ui.LWSaveLocations->setCurrentRow(row);
    }
}

void MainWindow::saveLocationsToFile()
{
    QFile file("locations.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);
    for (int i = 0; i < ui.LWSaveLocations->count(); i++)
        out << ui.LWSaveLocations->item(i)->text() << "\n";
    file.close();
}

void MainWindow::startNavigation()
{
    // TODO: implement navigation logic here
}

////////////////////////////////////////
////////////////////////////////////////

void MainWindow::initializeRViz() {
    // 1) Create RenderPanel and add to UI
    renderPanel_ = new rviz_common::RenderPanel();
    auto *layout = new QVBoxLayout(ui.rvizContainer);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(renderPanel_);

    // 2) Keep ROS node abstraction alive
    rviz_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");

    // 3) Create VisualizationManager
    rviz_common::WindowManagerInterface* wm = nullptr;
    auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    manager_ = new rviz_common::VisualizationManager(renderPanel_, rviz_ros_node_, wm, clock);

    // 4) Initialize RenderPanel and Manager
    renderPanel_->initialize(manager_);
    manager_->initialize();

    // 5) Set fixed frame before creating displays
    manager_->setFixedFrame("map");
    manager_->startUpdate();

    // 6) Configure view controller
    manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");

    auto tool_manager = manager_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));

    // 7) Defer display creation to allow Ogre scene to initialize
    QTimer::singleShot(100, this, [this]() {
        setupGridDisplay();
        setupTFDisplay();
        setupMapDisplay();
        setupRobotModelDisplay();
        setupLaserScanDisplay();
        setupMapSubscriber();
        qDebug() << "RViz displays initialized safely.";
    });
}


void MainWindow::setupGridDisplay() {
    // QString frame_id = frameLineEdit_->text();

    // Initialize the grid display
    grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    if (grid_) {
        grid_->subProp("Line Style")->setValue("Lines");
        grid_->subProp("Color")->setValue(QColor(Qt::white));
        // grid_->subProp("Reference Frame")->setValue(frame_id);
        // qDebug() << "Grid display configured for fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Grid display.";
    }
}

void MainWindow::setupTFDisplay() {
    // Set up the TF display to show frames with a fixed frame
    tf_display_ = manager_->createDisplay("rviz_default_plugins/TF", "TF Display", true);
    if (tf_display_) {
        tf_display_->subProp("Show Axes")->setValue(true);
        qDebug() << "TF display configured with axes and names shown.";
    } else {
        qDebug() << "Failed to create TF display.";
    }
}

void MainWindow::setupMapDisplay() {
    // QString frame_id = frameLineEdit_->text();

    // Set up the Map display for the /map topic
    map_display_ = manager_->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display_) {
        map_display_->subProp("Topic")->setValue("/map");
        map_display_->subProp("Alpha")->setValue(1.0);
        map_display_->subProp("Draw Behind")->setValue(false);
        map_display_->subProp("Color Scheme")->setValue("map");
        map_display_->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
        
        
        
        //map_display_->setEnabled(true);

        // qDebug() << "Map display configured for /map topic with fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Map display.";
    }
}



void MainWindow::setupRobotModelDisplay() {
    // Set up the RobotModel display for the /robot_description topic
    robot_model_display_ = manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    if (robot_model_display_) {
        robot_model_display_->subProp("Description Topic")->setValue("/tb3_0/robot_description");  // Set the topic to /robot_description
        robot_model_display_->subProp("TF Prefix")->setValue("");  // Set TF prefix to empty if needed /tb3_0/robot_description
        qDebug() << "RobotModel display configured for /robot_description topic.";
    } else {
        qDebug() << "Failed to create RobotModel display.";
    }
}



// void MainWindow::closeEvent(QCloseEvent *event) {
//     // Ensure clean shutdown of ROS 2 and close the application
//     rclcpp::shutdown();
//     event->accept();
//     qDebug() << "Application closed, ROS shutdown complete.";
// }

void MainWindow::setupMapSubscriber() {
    if (!rviz_ros_node_) {
        qDebug() << "rviz_ros_node_ not initialized, skipping map subscriber.";
        return;
    }
    auto node = rviz_ros_node_->get_raw_node();
    if (!node) {
        qDebug() << "rviz_ros_node_->get_raw_node() returned null, skipping map subscriber.";
        return;
    }
    mapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr /*msg*/) {
            mapReceived_ = true;
            qDebug() << "Map Received";
        }
    );
}

// void MainWindow::updateMapReceivedIndicator(bool received) {
//     if (received) {
//         mapReceivedIndicator_->setText("Map Received: Yes");
//         mapReceivedIndicator_->setStyleSheet("color: green;");
//     } else {
//         mapReceivedIndicator_->setText("Map Received: No");
//         mapReceivedIndicator_->setStyleSheet("color: red;");
//     }
// }

// Set up LaserScan Display
void MainWindow::setupLaserScanDisplay() {
    auto laser_scan_display = manager_->createDisplay("rviz_default_plugins/LaserScan", "LaserScan Display", true);
    if (laser_scan_display) {
        laser_scan_display->subProp("Topic")->setValue("/scan");       // Set to the topic where laser data is published
        laser_scan_display->subProp("Size (m)")->setValue(0.1);        // Adjust point size as needed
        laser_scan_display->subProp("Color")->setValue(QColor(Qt::green));  // Set color of laser points
        qDebug() << "LaserScan display configured successfully for /scan.";
    } else {
        qDebug() << "Failed to configure LaserScan display.";
    }
}
