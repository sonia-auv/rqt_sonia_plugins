import rospy

from controller_mission.srv import NameChange
from python_qt_binding.QtCore import QAbstractTableModel, Qt, QVariant


class ParameterTableModel(QAbstractTableModel):
    _current_state = None
    _headers = ['Parameter', 'Value']
    _headers_flags = [Qt.ItemIsSelectable | Qt.ItemIsEnabled,
                      Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled]

    def rowCount(self, index):
        if self._current_state:
            return len(self._current_state.parameters) + len(self._current_state.global_params)
        return 0

    def columnCount(self, index):
        return 2

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return QVariant(self._headers[col])
        return QVariant()

    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
            if row < len(self._current_state.parameters):
                if col == 0:
                    return self._current_state.parameters[row].variable_name
                elif col == 1:
                    return self._current_state.parameters[row].value
            else:
                row -= len(self._current_state.parameters)

                if col == 0:
                    return self._current_state.global_params[row].variable_name
                elif col == 1:
                    return self._current_state.global_params[row].value

    def setData(self, index, value, role):
        if role == Qt.EditRole and index.column() == 1:
            if len(value):
                name = self._current_state.name

                if index.row() < len(self._current_state.parameters):
                    try:
                        self._current_state.parameters[index.row()].value = float(value)
                    except ValueError:
                        self._current_state.parameters[index.row()].value = value
                else:
                    row = index.row() - len(self._current_state.parameters)
                    try:
                        self._current_state.global_params[row].value = float(value)
                    except ValueError:
                        self._current_state.global_params[row].value = value
                if name != self._current_state.name:
                    self._current_state.notify_name_changed(name, self._current_state.name)
                self.dataChanged.emit(index, index)
            return True

    def state_selection_changed(self, state, submission):
        if submission:
            self._headers = ['Global Parameter', 'Value']
        else:
            self._headers = ['Parameter', 'Value']
        self._current_state = state
        self.layoutChanged.emit()

    def flags(self, index):
        return self._headers_flags[index.column()]


class GlobalParameterTableModel(QAbstractTableModel):
    _current_global_params = None
    _headers = ['Global parameter', 'Value']
    _headers_flags = [Qt.ItemIsSelectable | Qt.ItemIsEnabled,
                      Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled]

    def rowCount(self, index):
        if self._current_global_params:
            return len(self._current_global_params)
        return 0

    def columnCount(self, index):
        return 2

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return QVariant(self._headers[col])
        return QVariant()

    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
            if col == 0:

                return self._current_global_params[row].variable_name
            elif col == 1:
                return self._current_global_params[row].value

    def setData(self, index, value, role):
        if role == Qt.EditRole and index.column() == 1:
            if len(value):
                try:
                    self._current_global_params[index.row()].value = float(value)
                except ValueError:
                    self._current_global_params[index.row()].value = value
                self.dataChanged.emit(index, index)
            return True

    def global_params_changed(self, global_params):
        self._current_global_params = global_params
        self.layoutChanged.emit()

    def flags(self, index):
        return self._headers_flags[index.column()]
    


