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
            self.mpcPlanner.setStyleSheet("background-color: green")
            self.mpcSingleWpts.setStyleSheet("background-color: red")
            self.softKill.setStyleSheet("background-color: red")
        elif mode == 11:
            self.mpcPlanner.setStyleSheet("background-color: red")
            self.mpcSingleWpts.setStyleSheet("background-color: green")
            self.softKill.setStyleSheet("background-color: red")
        elif mode  == 0:
            self.mpcPlanner.setStyleSheet("background-color: red")
            self.mpcSingleWpts.setStyleSheet("background-color: red")
            self.softKill.setStyleSheet("background-color: green")
    
    def set_target_reached(self, status):
        if status:
            self.targetReached.setStyleSheet("background-color: green")
        else:
            self.targetReached.setStyleSheet("background-color: red")

    def set_mode(self, mode):
        self.setModePublisher.publish(data=mode)

    def controller_info_callback(self, msg):
        self.set_buttons_style(msg.mpc_mode)
        self.set_target_reached(msg.target_reached)

