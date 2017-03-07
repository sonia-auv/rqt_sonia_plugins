import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from provider_kill_mission.msg import MissionSwitchMsg,KillSwitchMsg


class KillMissionWidget(QWidget):
    mission_received = pyqtSignal('bool')
    kill_received = pyqtSignal('bool')

    def __init__(self):
        super(KillMissionWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('KillMissionWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'KillMission.ui')
        loadUi(ui_file, self)

        self._mission_switch = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg, self._mission_switch_callback)
        self.mission_received.connect(self._handle_mission_result)

        self.kill_switch = rospy.Subscriber('/provider_kill_mission/kill_switch_msg',KillSwitchMsg, self._kill_switch_callback)
        self.kill_received.connect(self._handle_kill_result)

    def _mission_switch_callback(self, data):
        self.mission_received.emit(data.state)

    def _kill_switch_callback(self, data):
        self.kill_received.emit(data.state)

    def _handle_mission_result(self,mission):
        if mission:
            print 'in true'
            self.MissionSwitch_label.setPalette(self.paletteChecked.palette())
        else:
            self.MissionSwitch_label.setPalette(self.paletteUnchecked.palette())

    def _handle_kill_result(self,kill_switch_state):
        if kill_switch_state:
            self.KillSwitch_label.setPalette(self.paletteChecked.palette())
        else:
            self.KillSwitch_label.setPalette(self.paletteUnchecked.palette())




