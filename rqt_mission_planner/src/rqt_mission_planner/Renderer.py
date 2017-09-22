import math
import functools
import subprocess
import rospy
import yaml

import tkMessageBox
from Tkinter import Tk

from mission_model.state import Parameter
from controller_mission.srv import ReceivedState, ReceivedStateRequest, SendMission, SendMissionRequest
from python_qt_binding.QtGui import QPainter, QColor, QPen, QBrush
from python_qt_binding.QtWidgets import QAction, QMenu
from python_qt_binding.QtCore import QPointF, QRect, Qt


class TransitionUI:
    def __init__(self,name, state1, state2):
        self.name,self.state1, self.state2 = name,state1, state2

    def draw(self, painter):
        angle = math.atan2(self.state2.get_y() - self.state1.get_y(), self.state2.get_x() - self.state1.get_x())
        state1_position_x = self.state1.get_x() + (self.state1.radius / 2 * math.cos(angle))
        state1_position_y = self.state1.get_y() + (self.state1.radius / 2 * math.sin(angle))
        state2_position_x = self.state2.get_x() - (self.state2.radius / 2 * math.cos(angle))
        state2_position_y = self.state2.get_y() - (self.state2.radius / 2 * math.sin(angle))
        painter.drawLine(state1_position_x,
                         state1_position_y, state2_position_x,
                         state2_position_y)

        painter.drawLine(state2_position_x, state2_position_y,
                         state2_position_x - (20 * math.cos(angle + (math.pi / 4))),
                         state2_position_y - (20 * math.sin(angle + (math.pi / 4))))

        painter.drawLine(state2_position_x, state2_position_y,
                         state2_position_x - (20 * math.cos(angle - (math.pi / 4))),
                         state2_position_y - (20 * math.sin(angle - (math.pi / 4))))

        painter.drawText(state1_position_x + (state2_position_x - state1_position_x)/2 + 20*abs(math.sin(angle)),state1_position_y + (state2_position_y-state1_position_y)/2 + 20 *abs(math.cos(angle)),self.name)


#This is used while creating a transition to create a line following mouse.
class DummyTransition:
    def __init__(self, state, pos):
        self.state, self.pos = state, pos

    def draw(self, painter):
        angle = math.atan2(self.pos[1] - self.state.get_y(), self.pos[0] - self.state.get_x())
        state_position_x_ = self.state.get_x() + (self.state.radius / 2 * math.cos(angle))
        state_position_y = self.state.get_y() + (self.state.radius / 2 * math.sin(angle))
        mouse_position_x = self.pos[0]
        mouse_position_y = self.pos[1]
        painter.drawLine(state_position_x_,
                         state_position_y, mouse_position_x,
                         mouse_position_y)

        painter.drawLine(mouse_position_x, mouse_position_y,
                         mouse_position_x - (20 * math.cos(angle + (math.pi / 4))),
                         mouse_position_y - (20 * math.sin(angle + (math.pi / 4))))

        painter.drawLine(mouse_position_x, mouse_position_y,
                         mouse_position_x - (20 * math.cos(angle - (math.pi / 4))),
                         mouse_position_y - (20 * math.sin(angle - (math.pi / 4))))


class StateUI:
    radius = 1000
    PX_LETTER = 8

    def __init__(self, state, position):
        self.state = state
        self.position = position
        self.transitions = []
        state.subscribers.append(self.state_name_changed)
        self.calculate_radius()

    def state_name_changed(self, old, new):
        self.calculate_radius()

    def setPosition(self, x, y):
        self.position = (x - self.radius / 2, y - self.radius / 2)

    def add_transition(self, transition):
        self.state.add_transition_model(transition.name, transition.state2.state)
        self.transitions.append(transition)

    def remove_transition(self, transition):
        self.state.remove_transition_model(transition.name, transition.state2.state)
        self.transitions.remove(transition)

    def get_x(self):
        return self.position[0] + self.radius / 2

    def get_y(self):
        return self.position[1] + self.radius / 2

    def draw(self, painter):
        painter.drawEllipse(self.position[0], self.position[1], self.radius,
                            self.radius)
        painter.drawText(QRect(self.position[0], self.position[1], self.radius, self.radius), Qt.AlignCenter, self.state.name);

        for transition in self.transitions:
            transition.draw(painter)

    def calculate_radius(self):
        text_length_supposed = len(self.state.name) * self.PX_LETTER
        if text_length_supposed > self.radius:
            self.radius = text_length_supposed
        elif text_length_supposed < self.radius and text_length_supposed >= 100:
            self.radius = text_length_supposed
        elif text_length_supposed < 100:
            self.radius = 100

    def contains(self, pos, scale):
        return math.pow(pos.x() - self.get_x() * scale, 2) + math.pow(pos.y() - self.get_y() * scale, 2) < math.pow(
            self.radius * scale / 2, 2)


class Renderer:
    EDIT = 0
    TRANSITION = 1
    current_paint_mode = EDIT
    dummy_transition_line = None

    def __init__(self, paint_panel, controller_mission_directory, load_submission_in_other_tab_function):
        self.paint_panel = paint_panel
        self.paint_panel.paintEvent_original = self.paint_panel.paintEvent
        self.paint_panel.paintEvent = self.my_paint_event
        self.controller_mission_directory = controller_mission_directory
        self.load_submission_in_other_tab_function = load_submission_in_other_tab_function

        # Mouse event
        self.paint_panel.mousePressEvent = self.my_mouse_press_event
        self.paint_panel.mouseReleaseEvent = self.my_mouse_release_event
        self.paint_panel.mouseMoveEvent = self.my_mouse_move_event
        self.paint_panel.wheelEvent = self.my_wheel_event
        self.paint_panel_key_press_event_original = self.paint_panel.keyPressEvent
        self.paint_panel.keyPressEvent = self._handle_key_press_event
        self.mouse_pressed = False
        self.last_position_moved = None
        self.mouse_position_in_state = None
        self.scale_value = 1
        self.paint_panel.setMouseTracking(True)
        self.current_mouse_position = None
        self.current_selected_stateui = None
        self.current_selected_globalparam = None

        self.state_listeners = []
        
        self.missionContainer = []
        self.statesui = []
        self.globalparams = []
        self.painter = QPainter()

        self.missionContainer = MissionContainer

        self.transitions = []

        self.translated_position = QPointF()

        self.container = None

    def set_as_root_state(self):
        if self.current_selected_stateui:
            for stateui in self.statesui:
                stateui.state.is_root = False
            self.current_selected_stateui.state.is_root = True
            self.paint_panel.update()

    def _handle_key_press_event(self, event):
        if event.key() == Qt.Key_Delete:
            if self.current_selected_stateui:
                self.statesui.remove(self.current_selected_stateui)
                for state in self.statesui:
                    for transition in list(state.transitions):
                        if transition.state2 == self.current_selected_stateui:
                            state.transitions.remove(transition)
                    for transition in list(state.state.transitions):
                        if transition.state == self.current_selected_stateui.state.name:
                            state.state.transitions.remove(transition)
                self.current_selected_stateui = None
                self.paint_panel.update()
        return self.paint_panel_key_press_event_original(event)

    def generate_current_menu(self):
        if not self.current_selected_stateui:
            return None
        menu = QMenu(self.paint_panel)

        transition_menu = QMenu('Add transition', self.paint_panel)
        for outcome in self.current_selected_stateui.state.outcome_states:
            transition_menu.addAction(QAction(self.paint_panel.tr(outcome), self.paint_panel,
                                              triggered=functools.partial(self.create_transition, outcome,
                                                                          self.current_selected_stateui.state)))

        menu.addMenu(transition_menu)
        menu.addSeparator()
        transition_delete = QMenu('Remove Transition', self.paint_panel)
        for transition in self.current_selected_stateui.transitions:
            transition_delete.addAction(QAction(self.paint_panel.tr(transition.name + ' : ' + transition.state2.state.name),self.paint_panel,triggered=functools.partial(self.delete_transition,self.current_selected_stateui,transition)))
        menu.addMenu(transition_delete)

        menu.addSeparator()
        if self.current_selected_stateui.state.is_submission:
            action_open_sub_mission = QAction(self.paint_panel.tr('Open Sub-Mission from CM...'),self.paint_panel,triggered=functools.partial(self.load_submission_from_cm,self.current_selected_stateui))
            menu.addAction(action_open_sub_mission)
        else:
            action_open_file = QAction(self.paint_panel.tr('Open in Editor...'),self.paint_panel,triggered=functools.partial(self.open_state_file,self.current_selected_stateui))
            menu.addAction(action_open_file)
            action_push_state = QAction(self.paint_panel.tr('Push State Code to CM...'),self.paint_panel,triggered=functools.partial(self.push_state_to_CM,self.current_selected_stateui))
            menu.addAction(action_push_state)

        return menu

    def load_submission_from_cm(self, stateui):
        self.load_submission_in_other_tab_function(stateui.state.submission_file)
        pass

    def open_state_file(self,stateui):
        subprocess.call(["xdg-open", self.controller_mission_directory + stateui.state.base_file])

    def push_state_to_CM(self,stateui):
        try:
            with open(self.controller_mission_directory + stateui.state.base_file, 'r') as myfile:
                data = myfile.read()
            rospy.wait_for_service('mission_executor/push_state_content', timeout=2)
            push_state_content = rospy.ServiceProxy('mission_executor/push_state_content', ReceivedState)
            received_state_request = ReceivedStateRequest()
            received_state_request.name = stateui.state.base_file
            received_state_request.content = data
            push_state_content(received_state_request)

            root = Tk()
            root.withdraw()
            tkMessageBox.showinfo("Pushing state", "State pushed successfully")
        except rospy.ServiceException, e:
            print 'Mission Executor is not started'
            return
        pass

    def name_change(self, old_name, new_name):
        print self.statesui
        for state in self.statesui:
            for transition in list(state.state.transitions):
                if transition.state == old_name:
                    transition.state = new_name

    def delete_transition(self, stateui, transition):
        stateui.remove_transition(transition)
        self.paint_panel.update()

    def create_transition(self, outcome_name, state):
        self.current_paint_mode = self.TRANSITION
        self.current_transition = outcome_name
        self.paint_panel.setCursor(Qt.CrossCursor)

    def add_state(self, state):
        state = StateUI(state, (100, 100))
        self.statesui.append(state)
        self.paint_panel.update()

    def find_stateui_by_name(self, name):
        for stateui in self.statesui:
            if stateui.state.name == name:
                return stateui

        return None

    def my_wheel_event(self, mouseWheelEvent):
        self.scale_value += mouseWheelEvent.angleDelta().y() / 500.0
        if self.scale_value < 0.2:
            self.scale_value = 0.2
        elif self.scale_value > 5:
            self.scale_value = 4.0

        self.paint_panel.update()

    def my_mouse_press_event(self, mouseEvent):
        self.mouse_pressed = True
        current_mouse_position = mouseEvent.pos() - self.translated_position
        for state in self.statesui:
            if state.contains(current_mouse_position, self.scale_value):
                if self.current_paint_mode == self.TRANSITION:
                    self.current_selected_stateui.add_transition(TransitionUI(self.current_transition, self.current_selected_stateui, state))
                    self.dummy_transition_line = None
                    self.paint_panel.setCursor(Qt.ArrowCursor)
                    self.current_paint_mode = self.EDIT
                    return

                else:
                    self.current_selected_stateui = state
                    self._state_selection_changed(self.current_selected_stateui.state)
                    self.paint_panel.update()
                    return
                
        self.dummy_transition_line = None
        self.paint_panel.setCursor(Qt.ArrowCursor)
        self.current_paint_mode = self.EDIT
        self.current_selected_stateui = None
        self._state_selection_changed(None)
        self.paint_panel.update()

    def my_mouse_release_event(self, mouseEvent):
        if mouseEvent.button() == Qt.RightButton:
            menu = self.generate_current_menu()
            if menu:
                menu.exec_(self.paint_panel.mapToGlobal(mouseEvent.pos()))
        self.mouse_pressed = False
        self.last_position_moved = None

    def my_mouse_move_event(self, mouseEvent):
        if self.current_paint_mode == self.TRANSITION:
            # draw transition following mouse.
            self.dummy_transition_line = DummyTransition(self.current_selected_stateui,
                                                         (mouseEvent.x() / self.scale_value - self.translated_position.x(),
                                                          mouseEvent.y() / self.scale_value - self.translated_position.y()))
        else:
            if self.mouse_pressed:
                if self.last_position_moved and self.current_selected_stateui:
                    self.current_selected_stateui.setPosition(self.current_selected_stateui.get_x() + (
                        mouseEvent.x() - self.last_position_moved[0]), self.current_selected_stateui.get_y() + (
                                                                  mouseEvent.y() - self.last_position_moved[1]))
                elif self.last_position_moved:
                    self.translated_position = QPointF(
                        self.translated_position.x() + (mouseEvent.x() - self.last_position_moved[0]),
                        self.translated_position.y() + (mouseEvent.y() - self.last_position_moved[1]))
                    self.paint_panel.update()

                self.last_position_moved = (mouseEvent.x(), mouseEvent.y())

        self.current_mouse_position = mouseEvent.pos() - self.translated_position
        self.paint_panel.update()

    def my_paint_event(self, paintEvent):
        self.paint_panel.paintEvent_original(paintEvent)
        self.painter.begin(self.paint_panel)
        self.painter.setRenderHint(QPainter.Antialiasing)

        # Translate world
        self.painter.translate(self.translated_position)
        self.painter.scale(self.scale_value, self.scale_value)

        # Draw state
        self.painter.setPen(QColor(0, 0, 0))
        self.painter.setBrush(QColor(255, 255, 255))
        for stateui in list(self.statesui):
            if (self.current_selected_stateui and stateui == self.current_selected_stateui) or (
                                self.current_paint_mode == self.TRANSITION and self.current_mouse_position and stateui.contains(
                        self.current_mouse_position, self.scale_value)):
                self.painter.setPen(QPen(QBrush(QColor(0, 0, 255)), 3))
            else:
                self.painter.setPen(QColor(0, 0, 0))

            if stateui.state.is_root and stateui.state.is_submission:
                self.painter.setBrush(QColor(61, 203, 152))
            elif stateui.state.is_root:
                self.painter.setBrush(QColor(61, 163, 203))
            elif stateui.state.is_submission:
                self.painter.setBrush(QColor(109, 205, 61))
            else:
                self.painter.setBrush(QColor(255, 255, 255))
            stateui.draw(self.painter)

        if self.dummy_transition_line:
            self.dummy_transition_line.draw(self.painter)

        self.painter.end()

    def add_state_listener(self, listener):
        self.state_listeners.append(listener)

    def _state_selection_changed(self, state):
        for listener in list(self.state_listeners):
            listener.state_selection_changed(state)

    def add_global_parameters(self, name, value):
        self.globalparams.append(Parameter(name, value, ''))

    def create_container(self):
        return MissionContainer(self.statesui, self.globalparams)
        
        
class MissionContainer:
    def __init__(self,statesui, globalparams):
        self.statesui = statesui
        self.globalparams = globalparams

