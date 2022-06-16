import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QColor

from std_msgs.msg import UInt8
from sonia_common.msg import MpcInfo

class SetModeControlWidget(QWidget):

    def __init__(self):
        super(SetModeControlWidget, self).__init__()

        self.setObjectName('SetModeControlWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'set_control_mode.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')

        self.setModePublisher = rospy.Publisher('/proc_control/set_mode', UInt8, queue_size=10)

        self.controllerInfoSubscriber = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.controller_info_callback)

        # Subscribe to slot
        self.mpcPlanner.clicked[bool].connect(self.handle_mpc_planner_clicked)
        self.mpcSingleWpts.clicked[bool].connect(self.handle_mpc_single_wpts_clicked)
        self.softKill.clicked[bool].connect(self.handle_soft_kill_clicked)

        self.set_buttons_style(0)

    def handle_mpc_planner_clicked(self):
        self.set_buttons_style(10)
        self.set_mode(10)

    def handle_mpc_single_wpts_clicked(self):
        self.set_buttons_style(11)
        self.set_mode(11)

    def handle_soft_kill_clicked(self):
        self.set_buttons_style(0)
        self.set_mode(0)

    def set_buttons_style(self, mode):
        if mode == 10:
            self.mpcPlanner.setPalette(self.paletteChecked.palette())
            self.mpcSingleWpts.setPalette(self.paletteUnchecked.palette())
            self.softKill.setPalette(self.paletteUnchecked.palette())
        elif mode == 11:
            self.mpcPlanner.setPalette(self.paletteUnchecked.palette())
            self.mpcSingleWpts.setPalette(self.paletteChecked.palette())
            self.softKill.setPalette(self.paletteUnchecked.palette())
        elif mode  == 0:
            self.mpcPlanner.setPalette(self.paletteUnchecked.palette())
            self.mpcSingleWpts.setPalette(self.paletteUnchecked.palette())
            self.softKill.setPalette(self.paletteChecked.palette())

    def set_mode(self, mode):
        self.setModePublisher.publish(data=mode)

    def controller_info_callback(self, msg):
        self.set_buttons_style(msg.mpc_mode)
