import os
import rospy
import rospkg
import threading
from ThrusterAction import ThrusterAction

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QMainWindow

from sonia_msgs.msg import SendCanMsg



class ThrusterWidget(QMainWindow):

    def __init__(self):
        super(ThrusterWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('ThrusterControlWidget')


        self.request_device_id = SendCanMsg.DEVICE_ID_actuators
        self.request_unique_id = [SendCanMsg.UNIQUE_ID_ACT_port_motor, SendCanMsg.UNIQUE_ID_ACT_starboard_motor,
                                  SendCanMsg.UNIQUE_ID_ACT_back_depth_motor, SendCanMsg.UNIQUE_ID_ACT_front_depth_motor,
                                  SendCanMsg.UNIQUE_ID_ACT_back_heading_motor,
                                  SendCanMsg.UNIQUE_ID_ACT_front_heading_motor]
        self.request_method_number = SendCanMsg.METHOD_MOTOR_set_speed



        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thruster_control'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyThrusterControlWidget')

        # Subscribe to slot
        self.enableButton.setEnabled(False)
        self.enableButton.clicked[bool].connect(self._handle_enableButton_clicked)
        self.disableButton.clicked[bool].connect(self._handle_disableButton_clicked)
        self.actionStart_test.triggered.connect(self._handle_start_test_triggered)

        self.thruster_heading_front = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_front_heading_motor,'headingFront')
        self.thruster_heading_back = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_back_heading_motor,'headingBack')
        self.thruster_Starboard = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_starboard_motor,'Starboard')
        self.thruster_Port = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_port_motor,'Port')
        self.thruster_depthFront = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_front_depth_motor,'depthFront')
        self.thruster_depthBack = ThrusterAction(self,SendCanMsg.UNIQUE_ID_ACT_back_depth_motor,'depthBack')

        self.publisher = rospy.Publisher("/provider_can/send_can_msg", SendCanMsg, queue_size=10)

        #self.publisher.publish(self.request_device_id, self.request_unique_id[i], self.request_method_number,self.values[i])

    def _handle_enableButton_clicked(self, checked):
        self.enableButton.setEnabled(False)
        self.disableButton.setEnabled(True)
        self.depth.setEnabled(True)
        self.propulsion.setEnabled(True)
        self.heading.setEnabled(True)

    def _handle_disableButton_clicked(self, checked):
        self.disableButton.setEnabled(False)
        self.enableButton.setEnabled(True)
        self.depth.setEnabled(False)
        self.propulsion.setEnabled(False)
        self.heading.setEnabled(False)

    def _handle_start_test_triggered(self):
        t = threading.Thread(target=self._execute_test)
        t.start()

    def _execute_test(self):
        self.thruster_heading_front.run_test()
        self.thruster_heading_back.run_test()
        self.thruster_Starboard.run_test()
        self.thruster_Port.run_test()
        self.thruster_depthFront.run_test()
        self.thruster_depthBack.run_test()


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
