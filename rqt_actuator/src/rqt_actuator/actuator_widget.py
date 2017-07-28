from __future__ import division
import os
import rospkg
import rospy
import time
from std_msgs.msg import String
from smach_msgs.msg import SmachContainerStatus
from provider_actuators.srv import DoActionSrv, DoActionSrvRequest
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

        self.drop_port.clicked.connect(self._handle_drop_port)
        self.drop_starboard.clicked.connect(self._handle_drop_starboard)
        self.torpido_port.clicked.connect(self._handle_torpido_port)
        self.torpido_starboard.clicked.connect(self._handle_torpido_starboard)


    def _handle_drop_port(self):
        try:
            self.do_action_srv(DoActionSrvRequest.ELEMENT_DROPPER,DoActionSrvRequest.SIDE_PORT,DoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        except rospy.ServiceException, e:
            print e
            rospy.logerr('Controller Mission Node is not started')

    def _handle_drop_starboard(self):
        try:
            self.do_action_srv(DoActionSrvRequest.ELEMENT_DROPPER,DoActionSrvRequest.SIDE_STARBOARD,DoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        except rospy.ServiceException, e:
            print e
            rospy.logerr('Controller Mission Node is not started')
    def _handle_torpido_port(self):
        try:
            self.do_action_srv(DoActionSrvRequest.ELEMENT_TORPEDO,DoActionSrvRequest.SIDE_PORT,DoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        except rospy.ServiceException, e:
            print e
            rospy.logerr('Controller Mission Node is not started')

    def _handle_torpido_starboard(self):
        try:
            self.do_action_srv(DoActionSrvRequest.ELEMENT_TORPEDO,DoActionSrvRequest.SIDE_STARBOARD,DoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        except rospy.ServiceException, e:
            print e
            rospy.logerr('Controller Mission Node is not started')

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def shutdown_plugin(self):
        pass