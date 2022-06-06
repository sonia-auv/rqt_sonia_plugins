import os
import rospy
import rospkg
import threading
from .ThrusterAction import ThrusterAction

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow

from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class ThrusterWidget(QMainWindow):

    def __init__(self):
        super(ThrusterWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('ThrusterControlWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thruster_control'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyThrusterControlWidget')

        # Subscribe to slot
        self.enableButton.setEnabled(True)
        self.disableButton.setEnabled(False)
        self.resetPwmButton.setEnabled(False)
        self.enableButton.clicked[bool].connect(self._handle_enableButton_clicked)
        self.disableButton.clicked[bool].connect(self._handle_disableButton_clicked)
        self.actionStart_test.triggered.connect(self._handle_start_test_triggered)
        self.resetPwmButton.clicked[bool].connect(self._handle_resetPwmButton_clicked)

        self.thruster_1 = ThrusterAction(self, 0, 'T1')
        self.thruster_2 = ThrusterAction(self, 1, 'T2')
        self.thruster_3 = ThrusterAction(self, 2, 'T3')
        self.thruster_4 = ThrusterAction(self, 3, 'T4')
        self.thruster_5 = ThrusterAction(self, 4, 'T5')
        self.thruster_6 = ThrusterAction(self, 5, 'T6')
        self.thruster_7 = ThrusterAction(self, 6, 'T7')
        self.thruster_8 = ThrusterAction(self, 7, 'T8')

        self.thruster_publisher = rospy.Publisher("/provider_thruster/thruster_pwm", UInt16MultiArray, queue_size=10)
        self.enable_thrusters_publisher = rospy.Publisher("/provider_power/activate_all_motor", Bool, queue_size=10)
        self.dry_test_service = rospy.ServiceProxy('/provider_thruster/dry_test', Empty)

        self.enableButton.setEnabled(True)
        self.disableButton.setEnabled(False)
        self.T1_T2.setEnabled(False)
        self.T3_T4.setEnabled(False)
        self.T5_T6.setEnabled(False)
        self.T7_T8.setEnabled(False)
        self.actionStart_test.setEnabled(False)

        self.pwms = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    
    def _handle_resetPwmButton_clicked(self, checked):
        self.pwms = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.send_pwms()

    def _handle_enableButton_clicked(self, checked):
        self.enableButton.setEnabled(False)
        self.disableButton.setEnabled(True)
        self.T1_T2.setEnabled(True)
        self.T3_T4.setEnabled(True)
        self.T5_T6.setEnabled(True)
        self.T7_T8.setEnabled(True)
        self.resetPwmButton.setEnabled(True)
        self.actionStart_test.setEnabled(True)
        self.enable_thrusters_publisher.publish(data = True)

    def _handle_disableButton_clicked(self, checked):
        self.enableButton.setEnabled(True)
        self.disableButton.setEnabled(False)
        self.T1_T2.setEnabled(False)
        self.T3_T4.setEnabled(False)
        self.T5_T6.setEnabled(False)
        self.T7_T8.setEnabled(False)
        self.resetPwmButton.setEnabled(False)
        self.actionStart_test.setEnabled(False)
        self.enable_thrusters_publisher.publish(data = False)

    def set_pwm(self, index, value):
        self.pwms[index] = value

    def send_pwms(self):
        self.thruster_publisher.publish(data = self.pwms)

    def _handle_start_test_triggered(self):
        self.dry_test_service()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
