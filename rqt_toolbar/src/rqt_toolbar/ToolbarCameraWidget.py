import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

class CameraWidget(QWidget):

    def __init__(self):
        super(CameraWidget, self).__init__()
        self.setObjectName('CameraWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'start_cam.ui')
        loadUi(ui_file, self)

        self.btnBottom.clicked.connect(self.handle_bottom_click)
        self.btnFront.clicked.connect(self.handle_front_click)

        # TODO: We must to implement the ON/OFF cameras.
        self.btnBottom.setEnabled(False)
        self.btnFront.setEnabled(False)

    def handle_bottom_click(self):
        pass

    def handle_front_click(self):
        pass
