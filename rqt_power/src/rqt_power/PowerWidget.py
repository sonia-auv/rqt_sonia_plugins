import os
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtCore import pyqtSignal
from sonia_msgs.msg import PowerMsg, ActivateAllPS
from PowerCardButtonAction import PowerCardButtonAction


class PowerWidget(QMainWindow):
    power_result_received = pyqtSignal(powerMsg)

    CMD_PS_V16_1 = 0
    CMD_PS_V16_2 = 1
    CMD_PS_V12 = 2
    CMD_PS_C16_1 = 3
    CMD_PS_C16_2 = 4
    CMD_PS_C12 = 5
    CMD_PS_temperature = 6
    CMD_PS_VBatt = 7

    check_ps_16v_2 = 21
    check_ps_16v_1 = 20
    check_ps_12v = 19

    def __init__(self):
        super(PowerWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('PowerControlWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_power'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyPowerControlWidget')

        self._power_subscriber = rospy.Subscriber("/provider_power/power", powerMsg, self._power_callback)

        self.activate_all_ps = rospy.Publisher('/provider_power/activate_all_ps', activateAllPS, queue_size=100)

        self.power_result_received.connect(self.show_Data)

        self.EnableAll.setEnabled(True)
        self.EnableAll.clicked.connect(self._handle_out_enable_all_clicked)

        self.DisableAll.setEnabled(False)
        self.DisableAll.clicked.connect(self._handle_out_disable_all_clicked)

        # Subscribe to slot
        # --------------------------------------------------card1---------------------------------------------------------------#
        # ---------------------------------------------------------------------------------------------------------------------#
        self.card_1_buttons = PowerCardButtonAction(self, '0')
        # --------------------------------------------------card2--------------------------------------------------------------#
        # ---------------------------------------------------------------------------------------------------------------------#
        self.card_2_buttons = PowerCardButtonAction(self, '1')
        # --------------------------------------------------card3--------------------------------------------------------------#
        # ---------------------------------------------------------------------------------------------------------------------#
        self.card_3_buttons = PowerCardButtonAction(self, '2')
        # --------------------------------------------------card4--------------------------------------------------------------#
        # ---------------------------------------------------------------------------------------------------------------------#
        self.card_4_buttons = PowerCardButtonAction(self, '3')


    def _power_callback(self, data):
        self.power_result_received.emit(data)

    def show_Data(self, powerData):

        format_data = '{:.2f}'.format(powerData.data)

        if powerData.cmd == self.CMD_PS_V16_1:

            eval('self.Voltage161' + str(powerData.slave)).display(format_data)
            eval('self.Voltage161Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_V16_2:

            eval('self.Voltage162' + str(powerData.slave)).display(format_data)
            eval('self.Voltage162Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_V12:

            eval('self.Voltage12' + str(powerData.slave)).display(format_data)
            eval('self.Voltage12Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_C16_1:

            eval('self.Current161' + str(powerData.slave)).display(format_data)
            eval('self.Current161Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_C16_2:

            eval('self.Current162' + str(powerData.slave)).display(format_data)
            eval('self.Current162Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_C12:

            eval('self.Current12' + str(powerData.slave)).display(format_data)
            eval('self.Current12Card' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_temperature:

            eval('self.Temperature' + str(powerData.slave)).display(format_data)
            eval('self.TemperatureCard' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.CMD_PS_VBatt:

            eval('self.Battery' + str(powerData.slave)).display(format_data)
            eval('self.BatteryCard' + str(powerData.slave)).display(format_data)

        elif powerData.cmd == self.check_ps_12v:
            if powerData.data == 0:
                eval('self.OutEnable12' + str(powerData.slave)).setEnabled(True)
                eval('self.OutDisable12' + str(powerData.slave)).setEnabled(False)
            else:
                eval('self.OutEnable12' + str(powerData.slave)).setEnabled(False)
                eval('self.OutDisable12' + str(powerData.slave)).setEnabled(True)

        elif powerData.cmd == self.check_ps_16v_1:
            if powerData.data == 0:
                eval('self.OutEnable161' + str(powerData.slave)).setEnabled(True)
                eval('self.OutDisable161' + str(powerData.slave)).setEnabled(False)
            else:
                eval('self.OutEnable161' + str(powerData.slave)).setEnabled(False)
                eval('self.OutDisable161' + str(powerData.slave)).setEnabled(True)

        elif powerData.cmd == self.check_ps_16v_2:
            if powerData.data == 0:
                eval('self.OutEnable162' + str(powerData.slave)).setEnabled(True)
                eval('self.OutDisable162' + str(powerData.slave)).setEnabled(False)
            else:
                eval('self.OutEnable162' + str(powerData.slave)).setEnabled(False)
                eval('self.OutDisable162' + str(powerData.slave)).setEnabled(True)

    def _handle_out_enable_all_clicked(self):
        self._set_all_bus_state(1)
        self.DisableAll.setEnabled(True)
        self.EnableAll.setEnabled(False)

    def _handle_out_disable_all_clicked(self):
        self._set_all_bus_state(0)
        self.DisableAll.setEnabled(False)
        self.EnableAll.setEnabled(True)

    def _set_all_bus_state(self, state):
        activation = activateAllPS()
        activation.data = bool(state)
        for i in range(0, 4):
            activation.slave = i
            for j in range(1, 3):
                activation.bus = j
                self.activate_all_ps.publish(activation)

    def _handle_start_test_triggered(self):
        pass

    def _execute_test(self):
        pass

    def shutdown_plugin(self):
        self._power_subscriber.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


