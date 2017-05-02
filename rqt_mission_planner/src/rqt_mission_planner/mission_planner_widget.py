from __future__ import division
import os
import rospkg
import copy
import yaml
import rospy
import time
import tkMessageBox
from Tkinter import Tk

from tkFileDialog import asksaveasfilename, askopenfilename
from controller_mission.srv import ReceivedMission, ReceivedMissionRequest, SendMission, SendMissionRequest

from mission_model.state import fill_state_from_path, fill_submission_from_path
from mission_model.parameter_table_model import ParameterTableModel, GlobalParameterTableModel
from manage_mission_widget import SaveMissionWidget, LoadMissionWidget, LoadParameterWidget

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget
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
        self._tab_widget_ui_file = os.path.join(rp.get_path('rqt_mission_planner'), 'resource', 'tab_widget.ui')
        self.controller_mission_directory = os.path.join(rp.get_path('controller_mission'))
        self.mission_executor_mission_state_default_folder = os.path.join(rp.get_path('controller_mission'), 'missions')
        loadUi(ui_file, self)
        new_tab = QWidget()
        loadUi(self._tab_widget_ui_file,new_tab)
        self.state_directory = os.path.join(rp.get_path('controller_mission'), 'src', 'controller_mission', 'state')
        self.submission_directory = os.path.join(rp.get_path('controller_mission'), 'missions')
        self._handle_refresh_lists()

        self.table_model = ParameterTableModel()
        self.table_model.dataChanged.connect(self._table_model_data_changed)
        self.params_table.setModel(self.table_model)

        self.global_table_model = GlobalParameterTableModel()
        self.global_params_table.setModel(self.global_table_model)

        self.actionSave_as_local.triggered.connect(self._handle_save_as)
        self.actionLoad_local.triggered.connect(self._handle_load)
        self.actionSave_to_mission_controller_remote.triggered.connect(self._handle_save_remotely)
        self.actionLoad_from_mission_service_remote.triggered.connect(self._handle_load_remotely)
        self.actionNew_mission.triggered.connect(self._handle_create_new_mission)
        self.actionRefresh_lists.triggered.connect(self._handle_refresh_lists)
        self.actionAdd_parameter.triggered.connect(self._handle_add_parameter)

        self.tabWidget.currentChanged.connect(self._handle_tab_changed)
        self.tabWidget.tabCloseRequested.connect(self._handle_tab_close_requested)

        self.add_state.clicked.connect(self.handle_add_state)
        self.add_submission.clicked.connect(self.handle_add_submission)
        new_tab.rootState.clicked.connect(self.set_as_root_state)
        new_tab.my_renderer = Renderer(new_tab.paint_panel, self.controller_mission_directory, self._load_mission_in_new_tab)
        self.renderer = new_tab.my_renderer
        self.add_in_list = []
        self.current_selected_globalparam = None
        self.tabWidget.addTab(new_tab, 'Main')
        self.renderer.add_state_listener(self)

        self.i = 0

    def clean_tabs(self):
        while self.tabWidget.count() > 1:
            self.tabWidget.removeTab(self.tabWidget.count()-1)
        self.state_selection_changed(None)
        self.global_table_model.global_params_changed(None)

    def _handle_tab_close_requested(self, index):
        if index == 0:
            return
        self.tabWidget.removeTab(index)
        self.state_selection_changed(None)

    def _handle_tab_changed(self, index):
        self.renderer = self.tabWidget.currentWidget().my_renderer

    def _load_mission_in_new_tab(self, mission_name):
        new_tab = QWidget()
        loadUi(self._tab_widget_ui_file,new_tab)
        new_tab.rootState.clicked.connect(self.set_as_root_state)
        new_tab.my_renderer = Renderer(new_tab.paint_panel, self.controller_mission_directory, self._load_mission_in_new_tab)
        self.tabWidget.addTab(new_tab, mission_name)
        self.tabWidget.setCurrentWidget(new_tab)
        self.renderer = self.tabWidget.currentWidget().my_renderer
        self.renderer.add_state_listener(self)
        self._load_mission_remotely(mission_name)

    def _handle_refresh_lists(self):
        self.states = []
        self.load_states(self.state_directory)
        self.submissions = []
        self.load_submissions(self.submission_directory)

    def validate_mission_has_root_state(self):
        for stateui in self.renderer.statesui:
            if stateui.state.is_root:
                return True
        return False

    def valid_mission(self):
        if not self.validate_mission_has_root_state():
            root = Tk()
            root.withdraw()
            tkMessageBox.showerror("Root State", 'Please select a Root State !')
            return False
        return True

    def _table_model_data_changed(self, a, b):
        self.tabWidget.currentWidget().paint_panel.update()

    def _handle_create_new_mission(self):
        self.clean_tabs()
        self.renderer.statesui = []
        self.renderer.globalparams = []
        self.tabWidget.currentWidget().label_mission_name.setText('---')
        self.tabWidget.currentWidget().paint_panel.update()

    def _handle_save_remotely(self):
        if not self.valid_mission():
            return

        self.save_mission_widget = SaveMissionWidget()
        self.save_mission_widget.select_mission(self._save_mission_remotely)

    def _save_mission_remotely(self, mission_name):
        if not self.valid_mission():
            return

        try:
            rospy.wait_for_service('mission_executor/set_mission_content', timeout=2)
            set_mission_client = rospy.ServiceProxy('mission_executor/set_mission_content', ReceivedMission)
            received_mission_request = ReceivedMissionRequest()
            received_mission_request.name = mission_name

            received_mission_request.content = yaml.dump(self.renderer.create_container())
            set_mission_client(received_mission_request)
            self.tabWidget.currentWidget().label_mission_name.setText(mission_name)
            self.tabWidget.currentWidget().paint_panel.update()
        except rospy.ServiceException, e:
            print 'Mission Executor is not started'
            return

    def _handle_load_remotely(self):
        self.clean_tabs()
        self.save_mission_widget = LoadMissionWidget()
        self.save_mission_widget.select_mission(self._load_mission_remotely)
        pass

    def _handle_add_parameter(self):
        self.save_mission_widget = LoadParameterWidget()
        self.save_mission_widget.select_mission(self._add_global_param_remotely)
        
    def _add_global_param_remotely(self, value, name):
        self.renderer.add_global_parameters(name, value)
        self.global_table_model.global_params_changed(self.renderer.globalparams)

    def _load_mission_remotely(self, mission_name):

        try:
            rospy.wait_for_service('mission_executor/get_mission_content', timeout=2)
            get_mission_client = rospy.ServiceProxy('mission_executor/get_mission_content', SendMission)
            get_mission_request = SendMissionRequest()
            get_mission_request.name = mission_name
            mission_content = get_mission_client(get_mission_request)
            self.renderer.missionContainer = yaml.load(mission_content.content)
            self.renderer.statesui = self.renderer.missionContainer.statesui
            self.renderer.globalparams = self.renderer.missionContainer.globalparams
            self.global_table_model.global_params_changed(self.renderer.globalparams)
            self.tabWidget.currentWidget().label_mission_name.setText(mission_name)
            self.tabWidget.currentWidget().paint_panel.update()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def _handle_load(self):
        self.clean_tabs()
        # instantiate a Tk window
        root = Tk()
        root.withdraw()
        fin = askopenfilename(defaultextension='.yml', initialdir=self.mission_executor_mission_state_default_folder)
        if fin:
            with open(fin, 'r') as inputfile:
                self.renderer.missionContainer = yaml.load(inputfile)
                self.renderer.statesui = self.renderer.missionContainer.statesui
                self.renderer.globalparams = self.renderer.missionContainer.globalparams
                self.global_table_model.global_params_changed(self.renderer.globalparams)
                self.tabWidget.currentWidget().label_mission_name.setText(os.path.basename(fin))
                self.tabWidget.currentWidget().paint_panel.update()

    def _handle_save_as(self):
        root = Tk()
        root.withdraw()
        fout = asksaveasfilename(defaultextension='.yml', initialdir=self.mission_executor_mission_state_default_folder)
        if fout:
            with open(fout, 'w') as output:
                print "allo"
                yaml.dump(self.renderer.create_container(), output, default_flow_style=False)
                self.tabWidget.currentWidget().label_mission_name.setText(os.path.basename(fout))
                self.tabWidget.currentWidget().paint_panel.update()

    def load_states(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_states(file_path)
                continue
            state = fill_state_from_path(file_path, self.controller_mission_directory)
            if state:
                self.states.append(state)
        self.refresh_state_list()

    def load_submissions(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_states(file_path)
                continue
            submission = fill_submission_from_path(file_path, self.controller_mission_directory)
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
        self.table_model.state_selection_changed(state, 0)
        self.tabWidget.currentWidget().rootState.setEnabled(state is not None)
        if state and state.is_submission:
            self.table_model.state_selection_changed(state, 1)
            self.state_type_label.setText('Sub-Mission')
        elif state:
            self.state_type_label.setText(state._name)
        else:
            self.state_type_label.setText('---')

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
