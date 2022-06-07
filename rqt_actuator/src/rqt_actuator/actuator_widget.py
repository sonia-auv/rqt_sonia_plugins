from __future__ import division
import os
import rospkg
import rospy
import time
from sonia_common.srv import ActuatorDoActionSrv, ActuatorDoActionSrvRequest
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal


# main class inherits from the ui window class
class ActuatorWidget(QWidget):

    def __init__(self):
        super(ActuatorWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_actuator'), 'resource', 'mainWidget.ui')
        loadUi(ui_file, self)

        self.do_action_srv = rospy.ServiceProxy('/provider_actuators/do_action_srv', ActuatorDoActionSrv)
        self.drop_port.clicked.connect(self._handle_drop_port)
        self.drop_starboard.clicked.connect(self._handle_drop_starboard)
        self.torpido_port.clicked.connect(self._handle_torpido_port)
        self.torpido_starboard.clicked.connect(self._handle_torpido_starboard)
        self.open_arm.clicked.connect(self._handle_open_robotic_arm)
        self.close_arm.clicked.connect(self._handle_close_robotic_arm)


    def _handle_drop_port(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator is not started')

    def _handle_drop_starboard(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')
    def _handle_torpido_port(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')

    def _handle_torpido_starboard(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')

    def _handle_open_robotic_arm(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_OPEN,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')

    def _handle_close_robotic_arm(self):
        try:
            self.do_action_srv(ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_CLOSE,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def shutdown_plugin(self):
        pass