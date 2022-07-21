from __future__ import division
import os
from time import sleep
import rospkg
import rospy
import threading
from sonia_common.srv import ActuatorDoActionSrv, ActuatorDoActionSrvRequest
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


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
        self.torpedo_port.clicked.connect(self._handle_torpedo_port)
        self.torpedo_starboard.clicked.connect(self._handle_torpedo_starboard)
        self.open_arm.clicked.connect(self._handle_open_robotic_arm)
        self.close_arm.clicked.connect(self._handle_close_robotic_arm)


    def _handle_drop_port(self):
        for t in threading.enumerate():
            if t.name=="Actuator Drop Port":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH, self.drop_port)
        newThread.name = "Actuator Drop Port"
        newThread.start()

    def _handle_drop_starboard(self):
        for t in threading.enumerate():
            if t.name=="Actuator Drop Starboard":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_DROPPER,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_DROPPER_LAUNCH, self.drop_starboard)
        newThread.name = "Actuator Drop Starboard"
        newThread.start()

    def _handle_torpedo_port(self):
        for t in threading.enumerate():
            if t.name=="Actuator Torpedo Port":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_PORT,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH, self.torpedo_port)
        newThread.name = "Actuator Torpedo Port"
        newThread.start()

    def _handle_torpedo_starboard(self):
        for t in threading.enumerate():
            if t.name=="Actuator Torpedo Starboard":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_TORPEDO,ActuatorDoActionSrvRequest.SIDE_STARBOARD,ActuatorDoActionSrvRequest.ACTION_TORPEDO_LAUNCH, self.torpedo_starboard)
        newThread.name = "Actuator Torpedo Starbaord"
        newThread.start()

    def _handle_open_robotic_arm(self):
        for t in threading.enumerate():
            if t.name=="Actuator Open Arm":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_OPEN,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC, self.open_arm)
        newThread.name = "Actuator Open Arm"
        newThread.start()

    def _handle_close_robotic_arm(self):
        for t in threading.enumerate():
            if t.name=="Actuator Close Arm":
                return
        newThread = Threads(self.do_action_srv, ActuatorDoActionSrvRequest.ELEMENT_ARM,ActuatorDoActionSrvRequest.ARM_CLOSE,ActuatorDoActionSrvRequest.ACTION_ARM_EXEC, self.close_arm)
        newThread.name = "Actuator Close Arm"
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
    def __init__(self, do_action_srv, param1, param2, param3, button):
        super(Threads, self).__init__()
        self.do_action_srv = do_action_srv
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.button = button
    
    def run(self):
        try:
            self.button.setStyleSheet("background-color: yellow")
            result = self.do_action_srv(self.param1, self.param2, self.param3)
            if result.success:
                self.button.setStyleSheet("background-color: green")
            else:
                self.button.setStyleSheet("background-color: red")
            oldname = self.name
            self.name = f"{self.name} finished"
            sleep(5)
            for t in threading.enumerate():
                if t.name == oldname:
                    return
            self.button.setStyleSheet("")
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Actuator Node is not started')
