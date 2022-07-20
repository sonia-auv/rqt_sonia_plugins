from __future__ import division
import os
import rospkg
import rospy
import threading
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
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        newThread.start()

    def _handle_drop_starboard(self):
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH)
        newThread.start()

    def _handle_torpido_port(self):
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        newThread.start()

    def _handle_torpido_starboard(self):
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH)
        newThread.start()

    def _handle_open_robotic_arm(self):
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_OPEN,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC)
        newThread.start()

    def _handle_close_robotic_arm(self):
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_CLOSE,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC)
        newThread.start()

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def shutdown_plugin(self):
        pass


class Threads(threading.Thread):
    def __init__(self, do_action_srv, param1, param2, param3):
        super(Threads, self).__init__()
        self.do_action_srv = do_action_srv
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
    
    def run(self):
        try:
            self.do_action_srv(self.param1, self.param2, self.param3)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')
