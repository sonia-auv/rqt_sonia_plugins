from PyQt4.QtGui import QFrame


class FocusedFrame(QFrame):
    def __init__(self, parent=None):
        super(FocusedFrame, self).__init__(parent)
        self.setObjectName('ExecutionContainer')
