import os
import rospy
import rospkg
import tkMessageBox

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from python_qt_binding.QtCore import pyqtSignal

from provider_power.msg import powerMsg

class BatteryWidget(QWidget):

    BATT_MAX = 28
    BATT_THRESHOLD = 25.6
    psu_received = pyqtSignal(powerMsg)
    cmd_ps_vbatt = 7

    def __init__(self, no_batt, slave):
        super(BatteryWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('BatteryWidget')
        self.no_batt = no_batt
        self.bat_max = 16.8
        self.bat_min = 15.4
        self.bat_warning = 15.6

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'battery.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self._power_supply = rospy.Subscriber('/provider_power/power', powerMsg, self._power_supply_callback)
        self.psu_received.connect(self._handle_result)
        
        if no_batt == 1:
            self.battery_label.setText('Battery DVL :')
        else:
            self.battery_label.setText('Battery PC :')       
        
        self.nb_msg = 0
        self.time = None
        self.nb = 0

        self.slave = slave

    def _power_supply_callback(self, data):
        self.psu_received.emit(data)

    def _handle_result(self, msg):
        
        if msg.cmd == self.cmd_ps_vbatt and msg.slave == self.slave:
            self.progressBar.setValue(int(msg.data * 10))
            self.battery_value.setText('{:.2f} V'.format(msg.data))

            if msg.data <= self.bat_warning:
                self.nb+=1
            else:
                self.nb = 0

            percentage = (msg.data - self.bat_min) / (self.bat_max - self.bat_min) * 100
            if percentage >= 80:
                self.progressBar.setStyleSheet('selection-background-color:green ; background-color:gray ; color:black')
            elif 80 > percentage >= 50:
                self.progressBar.setStyleSheet('selection-background-color:yellow ; background-color:gray ; color:black')
            elif 50 > percentage >= 20:
                self.progressBar.setStyleSheet('selection-background-color:orange ; background-color:gray ; color:black')
            else:
                self.progressBar.setStyleSheet('selection-background-color:red ; background-color:gray ; color:black')
            
        if self.time is not None and rospy.get_time() - self.time >= 60:
            self.nb_msg = 0

        if msg.data <= self.bat_warning and self.nb_msg == 0 and self.nb == 5:
            self.nb = 0
            self.time = rospy.get_time()
            self.nb_msg = 1
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setWindowTitle("ATTENTION")
            msg.setStandardButtons(QMessageBox.Close)
            if self.no_batt == 1:
                msg.setText('Battery DVL : has a very low voltage')
            else:
                msg.setText('Battery PC has a very low voltage')
            msg.exec_()