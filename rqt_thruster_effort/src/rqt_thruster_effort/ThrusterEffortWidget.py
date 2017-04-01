import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from proc_control.msg import ThrusterEffort


class ThrusterEffortWidget(QWidget):

    monitor_thruster_msg = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(ThrusterEffortWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thruster_effort'), 'resource', 'mainwidget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Thruster Effort')

        self._thruster_subscriber = rospy.Subscriber('/proc_control/thruster_effort', ThrusterEffort,
                                                     self._handle_thruster_msg)

    def _handle_thruster_msg(self, msg):
        if msg.unique_id == ThrusterEffort.UNIQUE_ID_T1:
            self._set_thruster_value('T1', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T2:
            self._set_thruster_value('T2', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T3:
            self._set_thruster_value('T3', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T4:
            self._set_thruster_value('T4', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T5:
            self._set_thruster_value('T5', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T6:
            self._set_thruster_value('T6', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T7:
            self._set_thruster_value('T7', msg.parameter_value)
        elif msg.unique_id == ThrusterEffort.UNIQUE_ID_T8:
            self._set_thruster_value('T8', msg.parameter_value)

    def _set_thruster_value(self, thruster_name, value):
        eval('self.' + thruster_name + 'value').setText('{}'.format(int(value)) + ' %')
        eval('self.' + thruster_name + 'slider').setValue(int(value))


    def shutdown_plugin(self):
        self._can_subscriber.unregister()