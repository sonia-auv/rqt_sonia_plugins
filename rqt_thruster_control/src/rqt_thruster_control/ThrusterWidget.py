import os
import rospy
import rospkg
import threading
from ThrusterAction import ThrusterAction

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow

#from sonia_msgs.msg import SendCanMsg
from provider_thruster.msg import ThrusterEffort


class ThrusterWidget(QMainWindow):

    def __init__(self):
        super(ThrusterWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('ThrusterControlWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thruster_control'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyThrusterControlWidget')

        # Subscribe to slot
        self.enableButton.setEnabled(False)
        self.enableButton.clicked[bool].connect(self._handle_enableButton_clicked)
        self.disableButton.clicked[bool].connect(self._handle_disableButton_clicked)
        self.actionStart_test.triggered.connect(self._handle_start_test_triggered)

        self.thruster_1 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T1, 'T1')
        self.thruster_2 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T2, 'T2')
        self.thruster_3 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T3, 'T3')
        self.thruster_4 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T4, 'T4')
        self.thruster_5 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T5, 'T5')
        self.thruster_6 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T6, 'T6')
        self.thruster_7 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T7, 'T7')
        self.thruster_8 = ThrusterAction(self, ThrusterEffort.UNIQUE_ID_T8, 'T8')


        self.publisher = rospy.Publisher("/provider_thruster/thruster_effort", ThrusterEffort, queue_size=10)

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
        self.thruster_1.run_test()
        self.thruster_2.run_test()
        self.thruster_3.run_test()
        self.thruster_4.run_test()
        self.thruster_5.run_test()
        self.thruster_6.run_test()
        self.thruster_7.run_test()
        self.thruster_8.run_test()


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
