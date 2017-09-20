import math

from python_qt_binding.QtGui import QPainter, QColor, QPen, QBrush, QTransform
from python_qt_binding.QtCore import QRect, QLineF, QPointF


class Buoy:

    def __init__(self, mainwidget):
        self.mainwidget = mainwidget
        self.painter = QPainter()

    def paint_object(self, pos_x, pos_y, width, height, angle=0):
        self.painter.begin(self.mainwidget)
        self.painter.setRenderHint(QPainter.Antialiasing)
        self.painter.setPen(QColor(0, 0, 0))

        self.painter.setBrush(QBrush(QColor(100, 100, 255)))

        self.painter.drawEllipse(pos_x, pos_y, width, height)
        self.painter.end()


class Path:

    def __init__(self, mainwidget):
        self.mainwidget = mainwidget
        self.painter = QPainter()
        self.rec = QRect()
        self.t = QTransform()
        self.line = QLineF()
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.p4 = None

    def paint_object(self, pos_x, pos_y, width, height, angle):
        self.painter.begin(self.mainwidget)
        self.painter.setPen(QColor(0, 0, 0))
        self.painter.setBrush(QBrush(QColor(100, 100, 255)))

        self.line.setP1(QPointF(int(pos_x), int(pos_y)))
        self.line.setAngle(90)
        self.line.setLength(height)

        self.painter.drawLine(self.line)
        self.painter.end()

