import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from python_qt_binding.QtCore import pyqtSignal

from std_msgs.msg import Float64MultiArray

class BatteryWidget(QWidget):

    BATT_MAX = 28
    BATT_THRESHOLD = 25.6
    psu_received = pyqtSignal(Float64MultiArray)
    cmd_ps_vbatt = 7

    def __init__(self, store_index):
        super(BatteryWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('BatteryWidget')
        self.store_index = store_index
        self.bat_max = 16.8
        self.bat_min = 14.5
        self.bat_warning = 14.8

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'battery.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self._power_supply = rospy.Subscriber('/provider_power/voltage', Float64MultiArray, self._power_supply_callback)
        self.psu_received.connect(self._handle_result)
        
        if self.store_index == 8:
            self.battery_label.setText('Bat. 1 :')
        elif self.store_index == 9:
            self.battery_label.setText('Bat. 2 :')       

    def _power_supply_callback(self, data):
        self.psu_received.emit(data)

    def _handle_result(self, msg):
        self.battery_value.setText('{:.2f} V'.format(msg.data[self.store_index]))
        percentage = ((msg.data[self.store_index] - self.bat_min) / (self.bat_max - self.bat_min)) * 100
        self.progressBar.setValue(percentage)

        if percentage >= 80:
            self.progressBar.setStyleSheet('selection-background-color:green ; background-color:gray ; color:black')
        elif 80 > percentage >= 50:
            self.progressBar.setStyleSheet('selection-background-color:yellow ; background-color:gray ; color:black')
        elif 50 > percentage >= 20:
            self.progressBar.setStyleSheet('selection-background-color:orange ; background-color:gray ; color:black')
        else:
            self.progressBar.setStyleSheet('selection-background-color:red ; background-color:gray ; color:black')

        if msg.data[self.store_index] <= self.bat_warning:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setWindowTitle("ATTENTION")
            msg.setStandardButtons(QMessageBox.Close)
            if self.store_index == 8:
                msg.setText('Battery 1 has a very low voltage')
            elif self.store_index == 9:
                msg.setText('Battery 2 has a very low voltage')
            msg.exec_()