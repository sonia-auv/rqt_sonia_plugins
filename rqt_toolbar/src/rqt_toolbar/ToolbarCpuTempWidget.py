import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from sensor_msgs.msg import Temperature

class CpuTempWidget(QWidget):
    systemTemperatureReceived = pyqtSignal(Temperature)

    def __init__(self, topic, type):
        super(CpuTempWidget, self).__init__()
        self.setObjectName('CpuTempWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'temp_sensor.ui')
        loadUi(ui_file, self)

        self.temp_label.setText('{}:'.format(type))

        rospy.Subscriber(topic, Temperature, self.system_temperature_callback)
        self.systemTemperatureReceived.connect(self.handle_result)
        self.temp_value.setText("{0} C".format("?"))

    def system_temperature_callback(self, data):
        self.systemTemperatureReceived.emit(data)

    def handle_result(self, msg):
        self.temp_value.setText("{:.2f} C".format(msg.temperature))
