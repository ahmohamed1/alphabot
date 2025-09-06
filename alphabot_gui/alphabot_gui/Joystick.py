from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt, QPointF, QRectF, QLineF
from PyQt5.QtWidgets import QWidget

class Joystick(QWidget):
    def __init__(self, parent=None, main_window=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(150, 150)
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
        x_value = ((normVector.dx() / self.__maxDistance)) * -1
        y_value = ((normVector.dy() / self.__maxDistance)) * -1

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

