import os
import rospy
import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from proc_control.srv import SetControlMode


class ControlModeWidget(QWidget):

    PositionMode = 0
    VelocityMode = 1

    def __init__(self):
        super(ControlModeWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('ControlModeWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'control_mode.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyControlModeWidget')
        self._change_control_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)

        # Subscribe to slot
        self.PMode.clicked[bool].connect(self._handle_pmode_clicked)
        self.VMode.clicked[bool].connect(self._handle_vmode_clicked)

        self.PMode.click()
        self.VMode.click()

        self.PMode.setPalette(self.paletteChecked.palette())
        self.VMode.setPalette(self.paletteUnchecked.palette())

    def _handle_pmode_clicked(self, checked):
        if checked:
            self.PMode.setPalette(self.paletteChecked.palette())
            self.VMode.setPalette(self.paletteUnchecked.palette())
            self.change_control_mode(self.PositionMode)
        else:
            self.PMode.setPalette(self.paletteUnchecked.palette())
            self.VMode.setPalette(self.paletteChecked.palette())

    def _handle_vmode_clicked(self, checked):
        self.PMode.toggle()
        if checked:
            self.VMode.setPalette(self.paletteChecked.palette())
            self.PMode.setPalette(self.paletteUnchecked.palette())
            self.change_control_mode(self.VelocityMode)
        else:
            self.VMode.setPalette(self.paletteUnchecked.palette())
            self.PMode.setPalette(self.paletteChecked.palette())

    def change_control_mode(self, mode):
        try:
            self._change_control_mode(mode=mode)
        except rospy.ServiceException as err:
            rospy.logerr(err)
