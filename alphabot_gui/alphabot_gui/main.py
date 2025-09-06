#!/usr/bin/env python3

from PyQt5.QtWidgets import QMainWindow, QApplication

from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QGridLayout, QStyleFactory, QSlider, QLabel
from PyQt5.uic import loadUi
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMessageBox, QInputDialog, QLineEdit
from PyQt5.QtCore import QDir

from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtGui import QImage, QPixmap
import numpy as np

from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtGui import QBrush, QColor


from alphabot_gui.ros2Interface import ROS2Interface
from alphabot_gui.Joystick import Joystick

import sys
import threading
import rclpy

import sys

from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('alphabot_gui')
ui_file_path = os.path.join(package_share_directory, 'alphabot_gui.ui')


class MainUI(QMainWindow):
    def __init__(self, ros2_interface):
        super(MainUI, self).__init__()
        self.ros2_interface = ros2_interface

        package_share = get_package_share_directory('alphabot_gui')
        ui_path = os.path.join(package_share, 'alphabot_gui.ui')
        self.uiView = loadUi(ui_path, self)

        self.uiView.pbEmergencyStop.pressed.connect(self.emergencyPressed)

        # define the manual controll buttons
        self.uiView.PBForward.pressed.connect(lambda: self.manualtController("FORWARD"))
        self.uiView.PBBackWord.pressed.connect(lambda: self.manualtController("BACKWARD"))
        self.uiView.PBRight.pressed.connect(lambda: self.manualtController("RIGHT"))
        self.uiView.PBLeft.pressed.connect(lambda: self.manualtController("LEFT"))
        self.uiView.PBStop.pressed.connect(lambda: self.manualtController("STOP"))
        self.uiView.HSSpeed.valueChanged.connect(self.changeSpeed)
        # battery monitor
        # self.battery_level = 0.0
        # self.updateBatteryLevel(self.battery_level)
        self.battery_timer = QTimer(self)
        self.battery_timer.timeout.connect(self.periodicBatteryUpdate)
        self.battery_timer.start(3000)

        # Go to location tap
        self.uiView.PBAddNewLocation.pressed.connect(self.addLocation)
        self.uiView.PBEditLocation.pressed.connect(self.editLocation)
        self.uiView.PBStartNavigation.pressed.connect(self.startNavigation)
        # self.uiView.GVMap
        # self.uiView.LWSaveLocations

        # Build map tap
        self.uiView.PBStartMapping.pressed.connect(self.mapTap)
        self.uiView.PBSaveMap.pressed.connect(self.mapTap)
        self.uiView.PBPause.pressed.connect(self.mapTap)
        self.uiView.PBLoadMap.pressed.connect(self.mapTap)


        # Build joystick
        self.joystick = Joystick(self.uiView.GVController, self)

         # ✅ Setup the scene for GVMap
        self.map_scene = QGraphicsScene()
        self.uiView.GVMap.setScene(self.map_scene)
        self.map_item = None

        # ✅ Connect ROS2 map signal → PyQt slot
        self.ros2_interface.map_received.connect(self.update_map_view)
        self.uiView.GVMap.setScene(self.map_scene)

    def update_map_view(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Simple colors: free=white, occupied=black, unknown=gray
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == 0] = 255
        img[data == 100] = 0
        img[data == -1] = 127

        qimage = QImage(img.data, width, height, width, QImage.Format_Grayscale8)
        pixmap = QPixmap.fromImage(qimage)

        if self.map_item is None:
            self.map_item = QGraphicsPixmapItem(pixmap)
            self.map_scene.addItem(self.map_item)
        else:
            self.map_item.setPixmap(pixmap)

        # optional: fit the view
        self.uiView.GVMap.fitInView(self.map_item, mode=1)

    def update_twist_msg(self, values):
        """Handle joystick position updates"""
        x_value, y_value = values
        if abs(x_value) > 0.1 or abs(y_value) > 0.1:
            # Joystick is being moved significantly
            linear_speed = y_value  # Forward/backward
            angular_speed = x_value  # Rotation
            self.ros2_interface.publish_cmd_joystick(linear_speed, angular_speed)
        else:
            # Joystick is centered or near center
            self.ros2_interface.publish_cmd_joystick(0.0, 0.0)

    def periodicBatteryUpdate(self):
        battery_level = self.ros2_interface.get_battery_level()
        self.updateBatteryLevel(battery_level)

    def mapTap():
        pass
    
    def changeSpeed (self, value):
        self.uiView.LSpeed.setText(f'{value/100}')


    def updateColor(self, color):
        stylesheet = f"""
        QProgressBar {{
                    border: 2px solid #555;
                    border-radius: 10px;
                    text-align: center;
                    font: bold 14px;
                    color: black;
                    background-color: #eee;
                }}
        QProgressBar::chunk {{
                    background-color: {color};
                    border-radius: 10px;
                    margin: 1px;
        }}
        """
        self.uiView.PBBattryLevel.setStyleSheet(stylesheet)

    def updateBatteryLevel(self, value):
        value = int(value)
        if value > 50:
            self.updateColor(" #00cc44")
        elif value > 20:  # Between 20 and 50
           self.updateColor("yellow")
        else:  # Below 20
            self.updateColor("red")
        self.uiView.PBBattryLevel.setValue(value)

    def emergencyPressed(self):
        print("Emergency Button Pressed")

    def manualtController(self, status):
        speed = self.uiView.HSSpeed.value() / 100.0  # example: slider 0-100 scaled to 0.0-1.0
        self.ros2_interface.publish_cmd(status, speed)
    
    def addLocation(self):
        inputDialog = QInputDialog()
        text, ok = inputDialog.getText(self, "Enter the location name",
                                       "Location name:", QLineEdit.Normal)
        if ok and text:
            print(text)

        pose = self.ros2_interface.getRobotPositionInMap()
        print(pose)


    def editLocation(self):
        pass

    def startNavigation(self):
        x = self.uiView.TEX_coordinate.toPlainText()
        y = self.uiView.TEY_coordinate.toPlainText()
        theta = self.uiView.TETheta_coordinate.toPlainText()

        print(x)

def main():
    rclpy.init()
    ros2_interface = ROS2Interface()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros2_interface,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    ui = MainUI(ros2_interface)
    ui.show()
    app.exec_()

    ros2_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()