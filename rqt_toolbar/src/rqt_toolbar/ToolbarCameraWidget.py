import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from sonia_msgs.srv import StartStopMedia

class CameraWidget(QWidget):

    def __init__(self):
        super(CameraWidget, self).__init__()
        self.setObjectName('CpuTempWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'start_cam.ui')
        loadUi(ui_file, self)

        self.start_stop_media = rospy.ServiceProxy('/provider_vision/start_stop_camera', StartStopMedia)
        self.btnBottom.clicked.connect(self.handle_bottom_click)
        self.btnFront.clicked.connect(self.handle_front_click)

    def handle_bottom_click(self):
        try:
            self.start_stop_media('Front_GigE', 2)
            self.start_stop_media('Bottom_GigE',1)
        except rospy.ServiceException as e:
            rospy.logerr('Service start_stop_media did not respond')

    def handle_front_click(self):
        try:
            self.start_stop_media('Bottom_GigE',2)
            self.start_stop_media('Front_GigE', 1)
        except rospy.ServiceException as e:
            rospy.logerr('Service start_stop_media did not respond')



    def system_temperature_callback(self, data):
        self.systemTemperatureReceived.emit(data)

    def handle_result(self, msg):
        self.temp_value.setText("{0} C".format(msg.cpuTemp))
