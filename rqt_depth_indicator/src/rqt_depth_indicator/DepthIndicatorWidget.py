import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QColor
from python_qt_binding.QtCore import SIGNAL

from nav_msgs.msg import Odometry
from proc_control.srv import EnableControl
from sonia_msgs.msg import SendCanMsg


class DepthIndicatorWidget(QWidget):
    def __init__(self):
        super(DepthIndicatorWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_depth_indicator'), 'resource', 'mainwidget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Depth Indicator')

        self._odom_subscriber = rospy.Subscriber('/proc_navigation/odom', Odometry, self._odom_callback)

        self.connect(self, SIGNAL("odometry_received(QString)"), self._handle_result)

    def _odom_callback(self, data):
        self.emit(SIGNAL('odometry_received'), str(data.pose.pose.position.z))

    def _handle_result(self, z_depth):
        depth = int(float(z_depth) * 10)
        self.depthSilder.setValue(depth)
        self.depthValue.setText(str(depth / 10.0))

    def shutdown_plugin(self):
        self._odom_subscriber.unregister()
