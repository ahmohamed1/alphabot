#!/usr/bin/env python3

import sys
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt, QPointF, QRectF, QLineF
from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QGridLayout, QStyleFactory, QSlider, QLabel
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Joystick(QWidget):
    def __init__(self, parent=None, main_window=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(400, 200)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50
        self.main_window = main_window

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if limitLine.length() > self.__maxDistance:
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickPosition(self):
        if not self.grabCenter:
            return (0, 0)
        normVector = QLineF(self._center(), self.movingOffset)
        x_value = (normVector.dx() / self.__maxDistance) * self.main_window.rotSpeedSlider.value()
        y_value = (normVector.dy() / self.__maxDistance) * self.main_window.fwSpeedSlider.value()

        return (x_value, y_value)

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.update()
        self.main_window.update_twist_msg((0.0, 0.0))

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        self.main_window.update_twist_msg(self.joystickPosition())

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('Simple uNav Console')

        self.node = node
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.twist_sub = self.node.create_subscription(Twist, '/joint_stats', self.joint_stats_callback, 10)

        # Create main widget container
        cw = QWidget()
        self.setCentralWidget(cw)

        # Create and set widget layout
        layout = QGridLayout()
        cw.setLayout(layout)

        # Speed sliders
        self.fwSpeedSlider = QSlider(Qt.Vertical)
        self.fwSpeedSlider.setMinimum(0)
        self.fwSpeedSlider.setMaximum(5)
        self.fwSpeedSlider.setValue(1)
        layout.addWidget(self.fwSpeedSlider, 1, 0)

        self.fwSpeedLabel = QLabel("Fw speed max\n(m/sec)")
        layout.addWidget(self.fwSpeedLabel, 2, 0)

        self.fwSpeedValueLabel = QLabel("0.50")
        layout.addWidget(self.fwSpeedValueLabel, 3, 0)

        self.rotSpeedSlider = QSlider(Qt.Vertical)
        self.rotSpeedSlider.setMinimum(0)
        self.rotSpeedSlider.setMaximum(5)
        self.rotSpeedSlider.setValue(1)
        layout.addWidget(self.rotSpeedSlider, 1, 1)

        self.rotSpeedLabel = QLabel("Rot speed max\n(deg/sec)")
        layout.addWidget(self.rotSpeedLabel, 2, 1)

        self.rotSpeedValueLabel = QLabel("180")
        layout.addWidget(self.rotSpeedValueLabel, 3, 1)

        # Joystick widget
        self.joystick = Joystick(self, self)
        layout.addWidget(self.joystick, 1, 3, 3, 3)

        # Speed displays
        self.fwSpeedDisplayLabel = QLabel("Forward speed (m/sec)")
        layout.addWidget(self.fwSpeedDisplayLabel, 1, 2)

        self.fwSpeedDisplayValue = QLabel("0")
        layout.addWidget(self.fwSpeedDisplayValue, 2, 2)

        self.rotSpeedDisplayLabel = QLabel("Rotation speed (deg/sec)")
        layout.addWidget(self.rotSpeedDisplayLabel, 3, 2)

        self.rotSpeedDisplayValue = QLabel("0")
        layout.addWidget(self.rotSpeedDisplayValue, 4, 2)

        self.show()

    def update_twist_msg(self, position):
        twist = Twist()
        twist.linear.x = position[1]
        twist.angular.z = position[0]
        self.twist_pub.publish(twist)

    def joint_stats_callback(self, msg):
        self.fwSpeedDisplayValue.setText(str(msg.linear.x))
        self.rotSpeedDisplayValue.setText(str(msg.angular.z))

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('joystick_controller')
    app = QApplication([])
    app.setStyle(QStyleFactory.create("Cleanlooks"))
    mw = MainWindow(node)

    # Start Qt event loop
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
