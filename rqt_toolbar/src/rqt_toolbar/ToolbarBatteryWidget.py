import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
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
        self.setObjectName('BatteryWdiget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'battery.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self._power_supply = rospy.Subscriber('/provider_power/power', powerMsg, self._power_supply_callback)
        self.psu_received.connect(self._handle_result)

        self.battery_label.setText('Battery {} :'.format(no_batt))

        self.slave = slave

    def _power_supply_callback(self, data):
        self.psu_received.emit(data)

    def _handle_result(self, msg):
        if msg.cmd == self.cmd_ps_vbatt and msg.slave == self.slave:
            self.battery_slider.setValue(int(msg.data * 100))
            self.battery_value.setText('{:.2f} V'.format(msg.data))

