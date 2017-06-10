import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from provider_system.msg import SystemTemperature

class CpuTempWidget(QWidget):

    def __init__(self):
        super(CpuTempWidget, self).__init__()
        self.setObjectName('CpuTempWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'temp_sensor.ui')
        loadUi(ui_file, self)

        rospy.Subscriber('/provider_system/system_temperature', SystemTemperature, self.system_temperature_callback)

        self.temp_value.setText("{0} C".format("?"))


    def system_temperature_callback(self, data):
        self.temp_value.setText("{0} C".format(data.cpuTemp))