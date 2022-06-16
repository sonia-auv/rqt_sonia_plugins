import os
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtCore import pyqtSignal
from std_msgs.msg import Float64MultiArray, Bool
from .PowerCardButtonAction import PowerCardButtonAction


class PowerWidget(QMainWindow):
    voltage_result_received = pyqtSignal(Float64MultiArray)
    voltage12V_result_received = pyqtSignal(Float64MultiArray)
    current_result_received = pyqtSignal(Float64MultiArray)
    temperature_result_received = pyqtSignal(Float64MultiArray)

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

        self._voltage_subscriber = rospy.Subscriber("/provider_power/voltage", Float64MultiArray, self._voltage_callback)
        self._voltage12V_subscriber = rospy.Subscriber("/provider_power/voltage12V", Float64MultiArray, self._voltage12V_callback)
        self._current_subscriber = rospy.Subscriber("/provider_power/current", Float64MultiArray, self._current_callback)
        self._temperature_subscriber = rospy.Subscriber("/provider_power/temperature", Float64MultiArray, self._temperature_callback)

        self.activate_all_motor = rospy.Publisher('/provider_power/activate_all_motor', Bool, queue_size=100)

        self.voltage_result_received.connect(self.show_Voltage)
        self.current_result_received.connect(self.show_Current)
        self.voltage12V_result_received.connect(self.show_12V)
        self.temperature_result_received.connect(self.show_Temperature)

        self.EnableAll.setEnabled(True)
        self.EnableAll.clicked.connect(self._handle_out_enable_all_clicked)

        self.DisableAll.setEnabled(False)
        self.DisableAll.clicked.connect(self._handle_out_disable_all_clicked)


    def _voltage_callback(self, data):
        self.voltage_result_received.emit(data)

    def _voltage12V_callback(self, data):
        self.voltage12V_result_received.emit(data)

    def _current_callback(self, data):
        self.current_result_received.emit(data)

    def _temperature_callback(self, data):
        self.temperature_result_received.emit(data)

    def show_12V(self, data):
        pass

    def show_Temperature(self, data):
        for i in range(len(data.data)-2):
            format_data = '{:.2f}'.format(data.data[i])
            eval('self.TempM' + str(i+1)).display(format_data)
            eval('self.TempM' + str(i+1) + '_2').display(format_data)

        format_data = '{:.2f}'.format(data.data[len(data.data)-2])
        self.TempB1.display(format_data)
        self.TempB1_2.display(format_data)
        format_data = '{:.2f}'.format(data.data[len(data.data)-1])
        self.TempB2.display(format_data)
        self.TempB2_2.display(format_data)

    def show_Current(self, data):

        for i in range(len(data.data)-2):
            format_data = '{:.2f}'.format(data.data[i])
            eval('self.CurrentM' + str(i+1)).display(format_data)
            eval('self.CurrentM' + str(i+1) + '_2').display(format_data)

        format_data = '{:.2f}'.format(data.data[len(data.data)-2])
        self.CurrentB1.display(format_data)
        self.CurrentB1_2.display(format_data)
        format_data = '{:.2f}'.format(data.data[len(data.data)-1])
        self.CurrentB2.display(format_data)
        self.CurrentB2_2.display(format_data)

    def show_Voltage(self, data):

        for i in range(len(data.data)-2):
            format_data = '{:.2f}'.format(data.data[i])
            eval('self.VoltageM' + str(i+1)).display(format_data)
            eval('self.VoltageM' + str(i+1) + '_2').display(format_data)

        format_data = '{:.2f}'.format(data.data[len(data.data)-2])
        self.VoltageB1.display(format_data)
        self.VoltageB1_2.display(format_data)
        format_data = '{:.2f}'.format(data.data[len(data.data)-1])
        self.VoltageB2.display(format_data)
        self.VoltageB2_2.display(format_data)

        

    def _handle_out_enable_all_clicked(self):
        #self._set_all_bus_state(1)
        self.DisableAll.setEnabled(True)
        self.EnableAll.setEnabled(False)
        self.activate_all_motor.publish(True)

    def _handle_out_disable_all_clicked(self):
        #self._set_all_bus_state(0)
        self.DisableAll.setEnabled(False)
        self.EnableAll.setEnabled(True)
        self.activate_all_motor.publish(False)

    # def _set_all_bus_state(self, state):
    #     activation = activateAllPS()
    #     activation.data = bool(state)
    #     for i in range(0, 4):
    #         activation.slave = i
    #         for j in range(1, 3):
    #             activation.bus = j
    #             self.activate_all_ps.publish(activation)

    def _handle_start_test_triggered(self):
        pass

    def _execute_test(self):
        pass

    def shutdown_plugin(self):
        self._voltage_subscriber.unregister()
        self._current_subscriber.unregister()
        self._voltage12V_subscriber.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


