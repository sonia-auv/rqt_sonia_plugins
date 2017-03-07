from __future__ import division
import os
import rospkg
import copy
import yaml
import rospy
from Tkinter import Tk
from tkFileDialog import asksaveasfilename, askopenfilename
from controller_mission.srv import ReceivedMission, ReceivedMissionRequest, SendMission, SendMissionRequest

from mission_model.state import fill_state_from_path, fill_submission_from_path
from mission_model.parameter_table_model import ParameterTableModel
from manage_mission_widget import SaveMissionWidget, LoadMissionWidget

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from Renderer import Renderer


class StateListener:
    def state_selection_changed(self, state):
        print "Listener not implemented"


# main class inherits from the ui window class
class MissionPlannerWidget(QMainWindow, StateListener):
    def __init__(self):
        super(MissionPlannerWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_mission_planner'), 'resource', 'mainwindow.ui')
        self.mission_executor_mission_state_default_folder = os.path.join(rp.get_path('controller_mission'), 'missions')
        loadUi(ui_file, self)
        state_directory = os.path.join(rp.get_path('controller_mission'), 'src', 'controller_mission', 'state')
        submission_directory = os.path.join(rp.get_path('controller_mission'), 'missions')

        self.states = []
        self.load_states(state_directory)
        self.submissions = []
        self.load_submissions(submission_directory)

        self.table_model = ParameterTableModel()
        self.property_table.setModel(self.table_model)

        self.actionSave_as_local.triggered.connect(self._handle_save_as)
        self.actionLoad_local.triggered.connect(self._handle_load)
        self.actionSave_to_mission_controller_remote.triggered.connect(self._handle_save_remotely)
        self.actionLoad_from_mission_service_remote.triggered.connect(self._handle_load_remotely)

        self.add_state.clicked.connect(self.handle_add_state)
        self.add_submission.clicked.connect(self.handle_add_submission)
        self.rootState.clicked.connect(self.set_as_root_state)
        self.renderer = Renderer(self.paint_panel)
        self.renderer.add_state_listener(self)

    def _handle_save_remotely(self):
        self.save_mission_widget = SaveMissionWidget()
        self.save_mission_widget.select_mission(self._save_mission_remotely)

    def _save_mission_remotely(self, mission_name):

        try:
            rospy.wait_for_service('mission_executor/set_mission_content', timeout=2)
            set_mission_client = rospy.ServiceProxy('mission_executor/set_mission_content', ReceivedMission)
            received_mission_request = ReceivedMissionRequest()
            received_mission_request.name = mission_name
            received_mission_request.content = yaml.dump(self.renderer.statesui)
            set_mission_client(received_mission_request)
        except rospy.ServiceException, e:
            print 'Mission Executor is not started'
            return

    def _handle_load_remotely(self):
        self.save_mission_widget = LoadMissionWidget()
        self.save_mission_widget.select_mission(self._load_mission_remotely)
        pass

    def _load_mission_remotely(self, mission_name):

        try:
            rospy.wait_for_service('mission_executor/get_mission_content', timeout=2)
            get_mission_client = rospy.ServiceProxy('mission_executor/get_mission_content', SendMission)
            get_mission_request = SendMissionRequest()
            get_mission_request.name = mission_name
            mission_content = get_mission_client(get_mission_request)
            self.renderer.statesui = yaml.load(mission_content.content)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def _handle_load(self):
        # instantiate a Tk window
        root = Tk()
        root.withdraw()
        fin = askopenfilename(defaultextension='.yml', initialdir=self.mission_executor_mission_state_default_folder)
        if fin:
            with open(fin, 'r') as inputfile:
                self.renderer.statesui = yaml.load(inputfile)

    def _handle_save_as(self):
        fout = asksaveasfilename(defaultextension='.yml', initialdir=self.mission_executor_mission_state_default_folder)
        if fout:
            with open(fout, 'w') as output:
                yaml.dump(self.renderer.statesui, output, default_flow_style=False)

    def load_states(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_states(file_path)
                continue
            state = fill_state_from_path(file_path)
            if state:
                self.states.append(state)
        self.refresh_state_list()

    def load_submissions(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_states(file_path)
                continue
            submission = fill_submission_from_path(file_path)
            if submission:
                self.submissions.append(submission)
        self.refresh_submission_list()

    def refresh_state_list(self):
        self.list_states.clear()
        for state in self.states:
            self.list_states.addItem(state.name)

    def refresh_submission_list(self):
        self.list_submissions.clear()
        for submission in self.submissions:
            self.list_submissions.addItem(submission.name)

    def state_selection_changed(self, state):
        self.table_model.state_selection_changed(state)
        self.rootState.setEnabled(state is not None)

    def set_as_root_state(self):
        self.renderer.set_as_root_state()

    def handle_add_state(self):
        if self.list_states.currentItem():
            state_name = self.list_states.currentItem().text()
            self.renderer.add_state(copy.deepcopy(self.find_state_by_name(state_name)))

    def handle_add_submission(self):
        if self.list_submissions.currentItem():
            submission_name = self.list_submissions.currentItem().text()
            self.renderer.add_state(copy.deepcopy(self.find_submission_by_name(submission_name)))

    def find_state_by_name(self, name):
        for state in self.states:
            if state.name == name:
                return state

    def find_submission_by_name(self, name):
        for submission in self.submissions:
            if submission.name == name:
                return submission

    def save_settings(self, plugin_settings, instance_settings):
        None

    def restore_settings(self, plugin_settings, instance_settings):
        None

    def shutdown_plugin(self):
        print 'Shutting down'
