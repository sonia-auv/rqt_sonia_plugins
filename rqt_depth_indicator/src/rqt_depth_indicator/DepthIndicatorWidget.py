import os
import rospy
import rospkg
import math

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class DepthIndicatorWidget(QWidget):
    odometry_received = pyqtSignal('QString')
    def __init__(self):
        super(DepthIndicatorWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_depth_indicator'), 'resource', 'mainwidget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Depth Indicator')

        self._odom_subscriber = rospy.Subscriber('/proc_nav/auv_states', Odometry, self._odom_callback)

        self.odometry_received.connect(self._handle_result)

    def _odom_callback(self, data):
        self.odometry_received.emit(str(data.pose.pose.position.z))

    def _handle_result(self, z_depth):
        depth = int(float(z_depth) * 10)
        self.depthSlider.setValue(depth)
        self.depthValue.setText(str(depth / 10.0))

    def shutdown_plugin(self):
        self._odom_subscriber.unregister()
