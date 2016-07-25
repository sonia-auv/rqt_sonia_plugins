from PyQt4.QtGui import QScrollArea


class ParameterContainer(QScrollArea):
    def __init__(self, parent=None):
        super(ParameterContainer, self).__init__(parent)
        self.setObjectName('ExecutionContainer')
