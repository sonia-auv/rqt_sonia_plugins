from __future__ import division
import os
import rospkg
import rospy
from controller_mission.srv import ListMissions

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class SaveMissionWidget(QWidget):
    def __init__(self):
        super(SaveMissionWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_mission_planner'), 'resource', 'save_mission.ui')
        loadUi(ui_file, self)

        self.ok_button.clicked.connect(self._handle_ok_press)

    def select_mission(self,callback_function):
        try:
            list_missions_srv = rospy.ServiceProxy('mission_executor/list_missions', ListMissions )
            list_missions_resp = list_missions_srv()
            missions_list = list_missions_resp.missions.split(',')
            self.mission_names.clear()
            for mission_name in missions_list:
                self.mission_names.addItem(mission_name)

            self.callback_function = callback_function
            self.show()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def _handle_ok_press(self):
        if self.new_mission_name.text():
            mission_name = self.new_mission_name.text().strip()
        else:
            mission_name = self.mission_names.currentText()
        self.callback_function(mission_name)
        self.hide()


class LoadMissionWidget(QWidget):
    def __init__(self):
        super(LoadMissionWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_mission_planner'), 'resource', 'load_mission.ui')
        loadUi(ui_file, self)

        self.ok_button.clicked.connect(self._handle_ok_press)

    def select_mission(self,callback_function):
        try:
            list_missions_srv = rospy.ServiceProxy('mission_executor/list_missions', ListMissions )
            list_missions_resp = list_missions_srv()
            missions_list = list_missions_resp.missions.split(',')
            self.mission_names.clear()
            for mission_name in missions_list:
                self.mission_names.addItem(mission_name)

            self.callback_function = callback_function
            self.show()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def _handle_ok_press(self):
        mission_name = self.mission_names.currentText()
        self.callback_function(mission_name)
        self.hide()