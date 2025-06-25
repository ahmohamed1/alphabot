from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
from enum import Enum

import sys


class ManualControlDirections(Enum):
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    STOP = 0

class MainUI(QMainWindow):
    def __init__(self):
        super(MainUI, self).__init__()

        self.uiView = loadUi("alphabot_gui.ui", self)

        self.uiView.pbEmergencyStop.pressed.connect(self.emergencyPressed)

        # define the manual controll buttons
        self.uiView.PBForward.pressed.connect(lambda: self.manualtController(ManualControlDirections.FORWARD))
        self.uiView.PBBackWord.pressed.connect(lambda: self.manualtController(ManualControlDirections.BACKWARD))
        self.uiView.PBRight.pressed.connect(lambda: self.manualtController(ManualControlDirections.RIGHT))
        self.uiView.PBLeft.pressed.connect(lambda: self.manualtController(ManualControlDirections.LEFT))
        self.uiView.PBStop.pressed.connect(lambda: self.manualtController(ManualControlDirections.STOP))
        self.uiView.HSSpeed.valueChanged.connect(self.changeSpeed)
        # battery monitor
        self.updateBatteryLevel(90)

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
        match  status:
            case ManualControlDirections.FORWARD:
                print("Forward")
            case ManualControlDirections.BACKWARD:
                print("Backward")
            case ManualControlDirections.LEFT:
                print("left")
            case ManualControlDirections.RIGHT:
                print("right")
            case ManualControlDirections.STOP:
                print("Stop")
            case default:
                print("nothing")
    
    def addLocation(self):
        x = self.uiView.TEX_coordinate.toPlainText()
        y = self.uiView.TEY_coordinate.toPlainText()
        theta = self.uiView.TETheta_coordinate.toPlainText()
    
    def editLocation(self):
        pass

    def startNavigation(self):
        x = self.uiView.TEX_coordinate.toPlainText()
        y = self.uiView.TEY_coordinate.toPlainText()
        theta = self.uiView.TETheta_coordinate.toPlainText()

        print(x)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = MainUI()
    ui.show()
    app.exec_()