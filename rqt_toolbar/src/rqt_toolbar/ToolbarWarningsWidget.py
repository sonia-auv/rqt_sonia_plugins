import os
from time import sleep
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class WarningsWidget(QWidget):

    def __init__(self):
        super(WarningsWidget, self).__init__()
        self.setObjectName('WarningsWidget')
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'warnings.ui')
        loadUi(ui_file, self)
        self.motorIndicator.setStyleSheet("background-color: green")
        self.imuIndicator.setStyleSheet("background-color: green")
        self.dvlIndicator.setStyleSheet("background-color: green")

        ### tests
        self.motorIndicator.setStyleSheet("background-color: yellow")
        self.motorIndicator.setEnabled(True)
        self.motorIndicator.clicked.connect(self.openMotorMenu)
        self.imuIndicator.setStyleSheet("background-color: red")
        self.dvlIndicator.setStyleSheet("background-color: green")
        self.motorIndicator.setToolTip("Engine 3 and 4 are faulty")
        self.imuIndicator.setToolTip("No data feedback from IMU")

    def openMotorMenu(self):
        def command():
            os.system("rqt --standalone rqt_power")
        
        for t in threading.enumerate():
            if t.name=="Motor Windows":
                return
        newThread = threading.Thread(target=command, name="Motor Windows")
        newThread.start()
