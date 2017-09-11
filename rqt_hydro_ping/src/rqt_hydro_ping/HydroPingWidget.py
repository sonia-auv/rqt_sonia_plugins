import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from provider_hydrophone.msg import PingMsg as provider_hydro_ping_msg
from proc_hydrophone.msg import PingPose as proc_hydro_ping_msg


class HydroPingWidget(QWidget):
    monitor_hydro_ping_msg = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(HydroPingWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_hydro_ping'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Hydro Ping')

        self.provider_hydro_subscriber = rospy.Subscriber('/provider_hydrophone/provider_hydrophone', provider_hydro_ping_msg,)

        self._thruster_subscriber = rospy.Subscriber('/provider_thruster/thruster_effort', ThrusterEffort,
                                                     self._handle_thruster_msg)

        self.monitor_hydro_ping_msg.connect(self._received_thruster_msg)

    def _handle_thruster_msg(self, msg):
        pass
        pass

    def _received_thruster_msg(self, msg):
        pass

    def _set_thruster_value(self, thruster_name, value):
        #eval('self.' + thruster_name + 'value').setText('{}'.format(int(value)) + ' %')
        #eval('self.' + thruster_name + 'slider').setValue(int(value))
        pass

    def shutdown_plugin(self):
        #self._thruster_subscriber.unregister()
        pass