import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QColor
from python_qt_binding.QtCore import SIGNAL

from proc_control.srv import EnableControl
from sonia_msgs.msg import SendCanMsg


class ThrusterEffortWidget(QWidget):

    def __init__(self):
        super(ThrusterEffortWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thruster_effort'), 'resource', 'mainwidget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Thruster Effort')

        self._can_subscriber = rospy.Subscriber('/provider_can/send_can_msg', SendCanMsg,self._handle_can_msg)

        self.connect(self, SIGNAL("changeUI(PyQt_PyObject)"), self._monitoring_can_msg)

    def _handle_can_msg(self,msg):
        if msg.device_id <> SendCanMsg.DEVICE_ID_actuators:
            return
        if msg.unique_id > 6:
            return
        if msg.method_number <> SendCanMsg.METHOD_MOTOR_set_speed:
            return

        self.emit(SIGNAL("changeUI(PyQt_PyObject)"), msg)


    def _monitoring_can_msg(self, msg):
        if msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_front_heading_motor:
            self._set_thruster_value('heading_front_',msg.parameter_value)
        elif msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_back_heading_motor:
            self._set_thruster_value('heading_back_',msg.parameter_value)
        elif msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_starboard_motor:
            self._set_thruster_value('starboard_',msg.parameter_value)
        elif msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_port_motor:
            self._set_thruster_value('port_',msg.parameter_value)
        elif msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_front_depth_motor:
            self._set_thruster_value('depth_front_',msg.parameter_value)
        elif msg.unique_id == SendCanMsg.UNIQUE_ID_ACT_back_depth_motor:
            self._set_thruster_value('depth_back_',msg.parameter_value)

    def _set_thruster_value(self,thruster_name, value):
        eval('self.' + thruster_name + 'value').setText('{}'.format(int(value)) + ' %')
        eval('self.' + thruster_name + 'slider').setValue(int(value))


    def shutdown_plugin(self):
        self._can_subscriber.unregister()