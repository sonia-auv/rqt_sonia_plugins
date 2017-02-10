import re
import os

class Parameter:
    def __init__(self, variable_name, value, description):
        self.variable_name, self.value, self.description = variable_name, value, description


class Transition:
    def __init__(self, outcome_name, state_name):
        self.outcome, self.state = outcome_name, state_name


class State:
    def __init__(self, name):
        self._name = name
        self.parameters = []
        self.transitions = []
        self.outcome_states = []

    @property
    def name(self):
        for parameter in self.parameters:
            if parameter.variable_name == 'state_name':
                return parameter.value
        return self._name

    @name.setter
    def name(self, value):
        self._name = value
        for parameter in self.parameters:
            if parameter.variable_name == 'state_name':
                parameter.value = value

    def add_transition_model(self, outcome_name, state_name):
        self.transitions.append(Transition(outcome_name,state_name))

    def remove_transition_model(self, outcome_name, state_name):
        for transition in list(self.transitions):
            if transition.outcome == outcome_name and transition.state == state_name:
                self.transitions.remove(transition)

    def add_parameter(self, name, value, desc):
        self.parameters.append(Parameter(name, value, desc))


def fill_state_from_path(file):
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
        s = State(os.path.basename(file)[:-3] + '.'+class_name)
        s.add_parameter('state_name', class_name, 'state_name')
        if len(outcome_state) == 0:
            s.outcome_states = ['succeeded', 'aborted']
        else:
            s.outcome_states = outcome_state
        if len(parameters_description) and class_name:

            for state_parameter in parameters_description:
                tab = state_parameter.split(',')
                name, value, desc = tab
                s.add_parameter(name.strip("'"), value, desc)
        return s
