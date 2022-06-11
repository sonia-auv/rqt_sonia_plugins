import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QActionGroup
from python_qt_binding.QtCore import pyqtSignal, Qt

from std_msgs.msg import Bool

from sonia_msgs.srv import OverrideMissionSwitch, OverrideMissionSwitchRequest, SetMissionSwitch, \
    SetMissionSwitchRequest, GetOverrideMissionSwitch, GetMissionSwitch, GetKillSwitch


class KillMissionWidget(QWidget):
    mission_received = pyqtSignal('bool')
    kill_received = pyqtSignal('bool')

    def __init__(self):
        super(KillMissionWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('KillMissionWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'KillMission.ui')
        loadUi(ui_file, self)

        self._mission_switch = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', Bool,
                                                self._mission_switch_callback)
        self.mission_received.connect(self._handle_mission_result)

        self.kill_switch = rospy.Subscriber('/provider_kill_mission/kill_switch_msg', Bool,
                                            self._kill_switch_callback)
        self.kill_received.connect(self._handle_kill_result)

        self.mission_switch_widget_mouse_release_original = self.mission_switch_widget.mouseReleaseEvent
        self.mission_switch_widget.mouseReleaseEvent = self._mission_switch_mouseReleaseEvent

        self._create_contextual_menu()

        self.initialize_components()

    def initialize_components(self):
        try:
            # Init override status
            get_override_mission_switch_state = rospy.ServiceProxy(
                '/provider_kill_mission/get_override_mission_switch_state',
                GetOverrideMissionSwitch)
            override_state = get_override_mission_switch_state()
            self._override_action.setChecked(bool(override_state.state))
            self._mission_switch_on_action.setEnabled(bool(override_state.state))
            self._mission_switch_off_action.setEnabled(bool(override_state.state))
            get_mission_switch_state = rospy.ServiceProxy('provider_kill_mission/get_mission_switch_state',
                                                          GetMissionSwitch)
            #Init missions status
            mission_state = get_mission_switch_state()
            self._mission_switch_on_action.setChecked(bool(mission_state.state))
            self._mission_switch_off_action.setChecked(not bool(mission_state.state))
            self._handle_mission_result(bool(mission_state.state))

            #Init kill status
            get_kill_switch_state = rospy.ServiceProxy('provider_kill_mission/get_kill_switch_state',
                                                          GetKillSwitch)
            kill_state = get_kill_switch_state()
            self._handle_kill_result(bool(kill_state.state))
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Mission Executor is not starteddd')

    def _create_contextual_menu(self):
        self._menu = QMenu(self.mission_switch_widget)

        self._override_action = QAction(self.mission_switch_widget.tr("Override mission switch"), self, checkable=True,
                                        triggered=self._override_mission_switch)
        self._menu.addAction(self._override_action)
        self._override_action.setChecked(False)
        self._menu.addSeparator()

        self._mission_switch_on_action = QAction(self.mission_switch_widget.tr("On"), self, checkable=True,
                                                 triggered=self._override_mission_switch_on)
        self._menu.addAction(self._mission_switch_on_action)
        self._mission_switch_on_action.setEnabled(False)

        self._mission_switch_off_action = QAction(self.mission_switch_widget.tr("Off"), self, checkable=True,
                                                  triggered=self._override_mission_switch_off)
        self._menu.addAction(self._mission_switch_off_action)

        self.action_group = QActionGroup(self.mission_switch_widget)
        self.action_group.addAction(self._mission_switch_on_action)
        self.action_group.addAction(self._mission_switch_off_action)
        self._mission_switch_off_action.setEnabled(False)
        self._mission_switch_off_action.setChecked(True)

    def _override_mission_switch(self, checked):
        try:
            override_mission_srv = rospy.ServiceProxy('/provider_kill_mission/override_mission_switch',
                                                      OverrideMissionSwitch)
            override_mission_req = OverrideMissionSwitchRequest()
            override_mission_req.state = int(checked)
            override_mission_srv(override_mission_req)

            self._mission_switch_on_action.setEnabled(checked)
            self._mission_switch_off_action.setEnabled(checked)

        except rospy.ServiceException as e:
            rospy.logerr('provider Kill mission is not started')

    def _override_mission_switch_on(self):
        try:
            set_mission_srv = rospy.ServiceProxy('/provider_kill_mission/set_mission_switch', SetMissionSwitch)
            set_mission_req = SetMissionSwitchRequest()
            set_mission_req.state = 1
            set_mission_srv(set_mission_req)
        except rospy.ServiceException as e:
            rospy.logerr('provider Kill mission is not started')

    def _override_mission_switch_off(self):
        try:
            set_mission_srv = rospy.ServiceProxy('/provider_kill_mission/set_mission_switch', SetMissionSwitch)
            set_mission_req = SetMissionSwitchRequest()
            set_mission_req.state = 0
            set_mission_srv(set_mission_req)
        except rospy.ServiceException as e:
            rospy.logerr('provider Kill mission is not started')

    def _mission_switch_mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._menu.exec_(self.MissionSwitch_label.mapToGlobal(event.pos()))

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