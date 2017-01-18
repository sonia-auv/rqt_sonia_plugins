import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import SIGNAL


from proc_control.srv import EnableControl
from sonia_msgs.msg import PowerSupplyMsg


class BatteryWidget(QWidget):

    BATT_MAX = 28
    BATT_THRESHOLD = 25.6

    def __init__(self):
        super(BatteryWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('BatteryWdiget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'battery.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self._power_supply = rospy.Subscriber('/provider_can/power_supply_msgs', PowerSupplyMsg, self._power_supply_callback)
        self.connect(self, SIGNAL("psu_received(QString)"), self._handle_result)

    def _power_supply_callback(self, data):
        self.emit(SIGNAL('psu_received'), str(data.light_voltage))

    def _handle_result(self,light_voltage):
        voltage = float(light_voltage)
        self.battery_slider.setValue(int(voltage*10))
        self.battery_value.setText('{:.2f} V'.format(voltage))

