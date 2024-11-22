# joystick_controller.py
from PyQt5 import QtCore, QtGui, QtWidgets
from virtual_joystick_windows import Ui_Form  # assuming the generated UI file is saved as virtual_joystick_ui.py

class JoystickController(QtWidgets.QMainWindow, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # Connect the sliders to the corresponding methods
        self.speed_slider.valueChanged.connect(self.update_speed)
        self.speed_slider_2.valueChanged.connect(self.update_rotation)

        # Initialize the joystick graphics view
        self.joystick_scene = QtWidgets.QGraphicsScene()
        self.joystick_gv.setScene(self.joystick_scene)
        self.joystick_item = QtWidgets.QGraphicsEllipseItem(0, 0, 50, 50)
        self.joystick_scene.addItem(self.joystick_item)

        # Simulate joystick movement for testing
        self.joystick_item.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable)
        self.joystick_scene.installEventFilter(self)

    def eventFilter(self, source, event):
        if event.type() == QtCore.QEvent.GraphicsSceneMouseMove and source is self.joystick_scene:
            self.handle_joystick_movement(event.scenePos())
        return super().eventFilter(source, event)

    def update_speed(self, value):
        self.speed_display_label.setText(str(value))
        self.max_velocity_display.setText(str(value))

    def update_rotation(self, value):
        self.rotation_display_label_2.setText(str(value))
        self.max_rotation_display.setText(str(value))

    def handle_joystick_movement(self, pos):
        # Update joystick position
        self.joystick_item.setPos(pos)
        # Print out the position of joystick_item
        print(f"Joystick position: {self.joystick_item.pos()}")

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    controller = JoystickController()
    controller.show()
    sys.exit(app.exec_())