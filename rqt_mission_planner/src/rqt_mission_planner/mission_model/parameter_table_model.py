from python_qt_binding.QtCore import QAbstractTableModel, Qt, QVariant


class ParameterTableModel(QAbstractTableModel):
    _current_state = None
    _headers = ['Parameter', 'Value']
    _headers_flags = [Qt.ItemIsSelectable | Qt.ItemIsEnabled,
                      Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled]

    def rowCount(self, index):
        if self._current_state:
            return len(self._current_state.parameters)
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

                return self._current_state.parameters[row].variable_name
            elif col == 1:
                return self._current_state.parameters[row].value

    def setData(self, index, value, role):
        if role == Qt.EditRole and index.column() == 1:
            if len(value):
                try:
                    self._current_state.parameters[index.row()].value = float(value)
                except ValueError:
                    self._current_state.parameters[index.row()].value = value
                self.dataChanged.emit(index,index)
            return True

    def state_selection_changed(self, state):
        self._current_state = state
        self.layoutChanged.emit()

    def flags(self, index):
        return self._headers_flags[index.column()]
