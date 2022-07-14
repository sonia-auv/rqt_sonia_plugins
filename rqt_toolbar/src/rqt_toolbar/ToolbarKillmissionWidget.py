import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QActionGroup
from python_qt_binding.QtCore import pyqtSignal, Qt

from std_msgs.msg import Bool

class KillMissionWidget(QWidget):
    mission_received = pyqtSignal(Bool)
    kill_received = pyqtSignal(Bool)

    def __init__(self):
        super(KillMissionWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('KillMissionWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'KillMission.ui')
        loadUi(ui_file, self)

        self._mission_switch = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', Bool, self._mission_switch_callback)
        self.mission_received.connect(self._handle_mission_result)

        self.kill_switch = rospy.Subscriber('/provider_kill_mission/kill_switch_msg', Bool, self._kill_switch_callback)
        self.kill_received.connect(self._handle_kill_result)

    def _mission_switch_callback(self, data):
        self.mission_received.emit(data)

    def _kill_switch_callback(self, data):
        self.kill_received.emit(data)

    def _handle_mission_result(self, msg):
        if msg.data:
            self.MissionSwitch_label.setPalette(self.paletteChecked.palette())
        else:
            self.MissionSwitch_label.setPalette(self.paletteUnchecked.palette())

    def _handle_kill_result(self, msg):
        if msg.data:
            self.KillSwitch_label.setPalette(self.paletteChecked.palette())
        else:
            self.KillSwitch_label.setPalette(self.paletteUnchecked.palette())

    def shutdown_plugin(self):
        self._mission_switch.unregister()
        self.kill_switch.unregister()