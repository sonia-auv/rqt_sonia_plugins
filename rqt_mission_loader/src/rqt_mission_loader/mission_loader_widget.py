from __future__ import division
import os
import rospkg
import rospy
import time
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
from controller_mission.srv import ListMissions, LoadMission, LoadMissionRequest

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal


# main class inherits from the ui window class
class MissionPlannerWidget(QWidget):
    loaded_mission_name_received = pyqtSignal(str)
    started_mission_name_received = pyqtSignal(str)
    current_state_name_received = pyqtSignal(str)

    def __init__(self):
        super(MissionPlannerWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_mission_loader'), 'resource', 'mainWidget.ui')
        loadUi(ui_file, self)

        self.mission_started_subscriber = rospy.Subscriber("/mission_executor/started_mission_name", String,
                                                           self._handle_started_mission_name_changed)
        self.mission_loaded_subscriber = rospy.Subscriber("/mission_executor/mission_loaded_name", String,
                                                          self._handle_loaded_mission_name_changed)
        self.mission_loaded_subscriber = rospy.Subscriber("/mission_executor_server/smach/container_status", SmachContainerStatus,
                                                          self._handle_smach_container_status)

        self.load_mission_names()
        self.load_button.clicked.connect(self._handle_load_button)
        self.loaded_mission_name_received.connect(self._handle_loaded_mission_name_result)
        self.started_mission_name_received.connect(self._handle_started_mission_name_result)
        self.current_state_name_received.connect(self._handle_mission_state_result)

    def _handle_started_mission_name_changed(self, mission_name):
        self.started_mission_name_received.emit(mission_name.data)

    def _handle_loaded_mission_name_changed(self, mission_name):
        self.loaded_mission_name_received.emit(mission_name.data)

    def _handle_smach_container_status(self,smach_container_status):
        if smach_container_status.info == 'HEARTBEAT':
            return
        self.current_state_name_received.emit(str(smach_container_status.active_states))

    def _handle_mission_state_result(self,state_name):
        self.current_state_name_label.setText(state_name)


    def _handle_loaded_mission_name_result(self, mission_name):
        self.loaded_mission_name_label.setText(mission_name + '   ['+time.strftime("%H:%M:%S")+']')

    def _handle_started_mission_name_result(self, mission_name):
        self.started_mission_name_label.setText(mission_name + '   ['+time.strftime("%H:%M:%S")+']')

    def _handle_load_button(self):
        if not self.mission_names.currentText():
            return

        try:
            load_mission_srv = rospy.ServiceProxy('mission_executor/load_mission', LoadMission)
            load_mission_req = LoadMissionRequest()
            load_mission_req.mission = self.mission_names.currentText()
            load_mission_srv(load_mission_req)
        except rospy.ServiceException, e:
            rospy.logerr('Controller Mission Node is not started')

    def load_mission_names(self):
        try:
            list_missions_srv = rospy.ServiceProxy('mission_executor/list_missions', ListMissions)
            list_missions_resp = list_missions_srv()
            missions_list = list_missions_resp.missions.split(',')
            self.mission_names.clear()
            for mission_name in missions_list:
                self.mission_names.addItem(mission_name)
        except rospy.ServiceException, e:
            rospy.logerr('Controller Mission Node is not started')

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def shutdown_plugin(self):
        self.mission_loaded_subscriber.unregister()
        self.mission_started_subscriber.unregister()
        print 'Shutting down'
