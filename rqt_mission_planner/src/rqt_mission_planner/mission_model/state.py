import re
import os
import rospkg
import yaml

SUB_MISSION_FILE = 'SubMission_file'

STATE_NAME = 'state_name'


class Parameter:
    subscribers = []

    def __init__(self, variable_name, value, description):
        self.variable_name, self.value, self.description = variable_name, value, description
        if isinstance(self.value, basestring):
            self.value = self.value.replace('\'','').replace('"','')

class Transition:
    def __init__(self, outcome_name, state_name):
        self.outcome, self.state = outcome_name, state_name

    def state_name_changed(self, old_name, new_name):
        if self.state == old_name:
            self.state = new_name


class State:
    is_root = False
    is_submission = False
    global_params = []
    base_file = None
    subscribers = []
    

    def notify_name_changed(self, old_name, new_name):
        for subscriber in self.subscribers:
            subscriber(old_name, new_name)

    def __init__(self, name,base_file):
        self._name = name
        self.base_file = base_file
        self.parameters = []
        self.transitions = []
        self.outcome_states = []

    @property
    def submission_file(self):
        for parameter in self.parameters:
            if parameter.variable_name == SUB_MISSION_FILE:
                return parameter.value
        return self._name

    @submission_file.setter
    def submission_file(self, value):
        self._name = value
        for parameter in self.parameters:
            if parameter.variable_name == STATE_NAME:
                parameter.value = value

    @property
    def name(self):
        for parameter in self.parameters:
            if parameter.variable_name == STATE_NAME:
                return parameter.value
        return self._name

    @name.setter
    def name(self, value):
        self.notify_name_changed(self.name,value)
        self._name = value
        for parameter in self.parameters:
            if parameter.variable_name == STATE_NAME:
                parameter.value = value

    def add_transition_model(self, outcome_name, state):
        tran = Transition(outcome_name, state.name)
        state.subscribers.append(tran.state_name_changed)
        self.transitions.append(tran)

    def remove_transition_model(self, outcome_name, state):
        for transition in list(self.transitions):
            if transition.outcome == outcome_name and transition.state == state.name:
                self.transitions.remove(transition)

    def add_parameter(self, name, value, desc):
        self.parameters.append(Parameter(name, value, desc))


def fill_state_from_path(file, controller_mission_directory):
    pattern_classname = re.compile(r'^class\s+(\w+)')
    pattern_parameter = re.compile(r'^.*[#]{0}.*\(Parameter\((.*)\)\)')
    pattern_outcomes = re.compile(r'^\s*def get_outcomes\(self\):$')
    pattern_def = re.compile(r'^\s*def.*:$')
    pattern_brakets_content = re.compile(r'^[^#].*\[(.*)\]')
    parameters_description = []
    class_name = None
    outcome_state = []
    in_outcome_state_function = False
    with open(file) as f:
        content = f.readlines()
        for line in content:
            line = line.strip()
            if not class_name:
                cn = pattern_classname.match(line)
                if cn:
                    class_name = cn.group(1)
            if len(outcome_state) == 0:
                outcome_def = pattern_outcomes.match(line)
                if in_outcome_state_function:
                    outcome_result = pattern_brakets_content.match(line)
                    if in_outcome_state_function and outcome_result:
                        # remove return + space than locate stuff between bracket []
                        result = outcome_result.group(1)
                        outcome_state = result.split(',')
                        outcome_state = [x.strip().strip("'") for x in outcome_state]
                    outside = pattern_def.match(line)
                    if outside:
                        in_outcome_state_function = False

                if outcome_def:
                    in_outcome_state_function = True
            m = pattern_parameter.match(line)
            if m:
                parameters_description.append(m.group(1))
    if class_name:
        s = State(os.path.basename(file)[:-3] + '.' + class_name, file.replace(controller_mission_directory,''))
        s.add_parameter('state_name', class_name, 'state_name')
        if len(outcome_state) == 0:
            s.outcome_states = ['succeeded', 'aborted']
        else:
            s.outcome_states = outcome_state
        if len(parameters_description) and class_name:

            for state_parameter in parameters_description:
                tab = state_parameter.split(',')
                name, value, desc = tab
                try:
                    s.add_parameter(name.strip("'"), float(value), desc)
                except ValueError:
                    s.add_parameter(name.strip("'"), value.strip(), desc)
        return s


def fill_submission_from_path(file, controller_mission_directory):
    rp = rospkg.RosPack()
    sub_mission_name = os.path.basename(file)[:-4]
    if sub_mission_name:
        filename_with_directory = file.replace(controller_mission_directory,'').replace('/missions/','')
        s = State(filename_with_directory[:-4], file.replace(controller_mission_directory,''))

        with open(file, 'r') as inputfile:
            mission_container = yaml.load(inputfile)
            if hasattr(mission_container, 'globalparams'):
                s.global_params = mission_container.globalparams
        s.is_submission = True
        s.add_parameter(STATE_NAME, sub_mission_name, '%s' % STATE_NAME)
        s.add_parameter(SUB_MISSION_FILE, filename_with_directory, '%s' % SUB_MISSION_FILE)
        s.outcome_states = ['succeeded', 'aborted']
        return s
