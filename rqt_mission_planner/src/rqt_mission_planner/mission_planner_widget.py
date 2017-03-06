from __future__ import division
import os
import rospkg
import re
import ast
import inspect
import math
import copy
import yaml

from mission_model.state import fill_state_from_path
from mission_model.parameter_table_model import ParameterTableModel

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QPoint
from python_qt_binding.QtWidgets import QAction, QMenu, QMainWindow, QTableWidgetItem

import rospy
from Renderer import Renderer


class StateListener:
    def state_selection_changed(self, state):
        print "Listener not implemented"


# main class inherits from the ui window class
class MissionPlannerWidget(QMainWindow, StateListener):
    def __init__(self, plugin):
        super(MissionPlannerWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_mission_planner'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)
        state_directory = os.path.join(rp.get_path('controller_mission'), 'src', 'controller_mission', 'state')
        self.states = []
        self.load_states(state_directory)
        self.table_model = ParameterTableModel()
        self.property_table.setModel(self.table_model)
        self.actionSave_as.triggered.connect(self._handle_save_as)
        self.actionLoad.triggered.connect(self._handle_load)

        self.add_state.clicked.connect(self.handle_add_state)
        self.rootState.clicked.connect(self.set_as_root_state)
        self.renderer = Renderer(self.paint_panel)
        self.renderer.add_state_listener(self)

    def _handle_load(self):
        with open('data.yml', 'r') as inputfile:
            self.renderer.statesui = yaml.load(inputfile)

    def _handle_save_as(self):
        with open('data.yml', 'w') as outfile:
            yaml.dump(self.renderer.statesui, outfile, default_flow_style=False)

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

    def refresh_state_list(self):
        self.list_states.clear()
        for state in self.states:
            self.list_states.addItem(state.name)

    def state_selection_changed(self, state):
        self.table_model.state_selection_changed(state)
        self.rootState.setEnabled(state is not None)

    def set_as_root_state(self):
        self.renderer.set_as_root_state()


    def handle_add_state(self):
        if self.list_states.currentItem():
            state_name = self.list_states.currentItem().text()
            self.renderer.add_state(copy.deepcopy(self.find_state_by_name(state_name)))

    def find_state_by_name(self, name):
        for state in self.states:
            if state.name == name:
                return state

    def save_settings(self, plugin_settings, instance_settings):
        None

    def restore_settings(self, plugin_settings, instance_settings):
        None

    def shutdown_plugin(self):
        print 'Shutting down'
