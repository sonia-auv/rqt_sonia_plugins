import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

class CpuTempWidget(QWidget):

    def __init__(self):
        super(CpuTempWidget, self).__init__()
        self.setObjectName('CpuTempWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'temp_sensor.ui')
        loadUi(ui_file, self)


        self.temp_value.setText("{0} C".format(25))