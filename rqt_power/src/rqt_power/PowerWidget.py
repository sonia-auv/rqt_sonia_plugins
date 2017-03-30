import os
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtCore import pyqtSignal
from provider_power.msg import powerMsg



class PowerWidget(QMainWindow):
    power_result_received = pyqtSignal(powerMsg)
    def __init__(self):
        super(PowerWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('PowerControlWidget')



        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_power'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyPowerControlWidget')

        self._power_subscriber = rospy.Subscriber("/provider_power/power", powerMsg, self._power_callback)

        self.power_result_received.connect(self._power_result_received)

        # Subscribe to slot
#--------------------------------------------------card1---------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
        self.OutEnable1611.setEnabled(True)
        self.OutEnable1611.clicked[bool].connect(self._handle_enableButton_16_1_clicked_Card1)
        self.OutDisable1611.setEnabled(False)
        self.OutDisable1611.clicked[bool].connect(self._handle_disableButton_16_1_clicked_Card1)

        self.OutEnable1621.setEnabled(True)
        self.OutEnable1621.clicked[bool].connect(self._handle_enableButton_16_2_clicked_Card1)
        self.OutDisable1621.setEnabled(False)
        self.OutDisable1621.clicked[bool].connect(self._handle_disableButton_16_2_clicked_Card1)

        self.OutEnable121.setEnabled(True)
        self.OutEnable121.clicked[bool].connect(self._handle_enableButton_12_clicked_Card1)
        self.OutDisable121.setEnabled(False)
        self.OutDisable121.clicked[bool].connect(self._handle_disableButton_12_clicked_Card1)
# --------------------------------------------------card2--------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
        self.OutEnable1612.setEnabled(True)
        self.OutEnable1612.clicked[bool].connect(self._handle_enableButton_16_1_clicked_Card2)
        self.OutDisable1612.setEnabled(False)
        self.OutDisable1612.clicked[bool].connect(self._handle_disableButton_16_1_clicked_Card2)

        self.OutEnable1622.setEnabled(True)
        self.OutEnable1622.clicked[bool].connect(self._handle_enableButton_16_2_clicked_Card2)
        self.OutDisable1622.setEnabled(False)
        self.OutDisable1622.clicked[bool].connect(self._handle_disableButton_16_2_clicked_Card2)

        self.OutEnable122.setEnabled(True)
        self.OutEnable122.clicked[bool].connect(self._handle_enableButton_12_clicked_Card2)
        self.OutDisable122.setEnabled(False)
        self.OutDisable122.clicked[bool].connect(self._handle_disableButton_12_clicked_Card2)
# --------------------------------------------------card3--------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
        self.OutEnable1613.setEnabled(True)
        self.OutEnable1613.clicked[bool].connect(self._handle_enableButton_16_1_clicked_Card3)
        self.OutDisable1613.setEnabled(False)
        self.OutDisable1613.clicked[bool].connect(self._handle_disableButton_16_1_clicked_Card3)

        self.OutEnable1623.setEnabled(True)
        self.OutEnable1623.clicked[bool].connect(self._handle_enableButton_16_2_clicked_Card3)
        self.OutDisable1623.setEnabled(False)
        self.OutDisable1623.clicked[bool].connect(self._handle_disableButton_16_2_clicked_Card3)

        self.OutEnable123.setEnabled(True)
        self.OutEnable123.clicked[bool].connect(self._handle_enableButton_12_clicked_Card3)
        self.OutDisable123.setEnabled(False)
        self.OutDisable123.clicked[bool].connect(self._handle_disableButton_12_clicked_Card3)
# --------------------------------------------------card4--------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
        self.OutEnable1614.setEnabled(True)
        self.OutEnable1614.clicked[bool].connect(self._handle_enableButton_16_1_clicked_Card4)
        self.OutDisable1614.setEnabled(False)
        self.OutDisable1614.clicked[bool].connect(self._handle_disableButton_16_1_clicked_Card4)

        self.OutEnable1624.setEnabled(True)
        self.OutEnable1624.clicked[bool].connect(self._handle_enableButton_16_2_clicked_Card4)
        self.OutDisable1624.setEnabled(False)
        self.OutDisable1624.clicked[bool].connect(self._handle_disableButton_16_2_clicked_Card4)

        self.OutEnable124.setEnabled(True)
        self.OutEnable124.clicked[bool].connect(self._handle_enableButton_12_clicked_Card4)
        self.OutDisable124.setEnabled(False)
        self.OutDisable124.clicked[bool].connect(self._handle_disableButton_12_clicked_Card4)

# --------------------------------------------------card1---------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
    def _power_callback(self, data):
        self.power_result_received.emit(data)


    def _power_result_received(self, powerData):
        self.Temperature1.display(powerData.temperature1)
        self.Current1611.display(powerData.cur16Pin1card1)
        self.Current1621.display(powerData.cur16Pin2card1)
        self.Current121.display(powerData.cur12card1)
        self.Voltage1611.display(powerData.volt16Pin1card1)
        self.Current1621.display(powerData.volt16Pin2card1)
        self.Current121.display(powerData.volt12card1)
        self.Battery1.display(powerData.battery1)
        self.Temperature2.display(powerData.temperature2)
        self.Current1612.display(powerData.cur16Pin1card2)
        self.Current1622.display(powerData.cur16Pin2card2)
        self.Current122.display(powerData.cur12card2)
        self.Voltage1612.display(powerData.volt16Pin1card2)
        self.Current1622.display(powerData.volt16Pin2card2)
        self.Current122.display(powerData.volt12card2)
        self.Battery2.display(powerData.battery2)
        self.Temperature3.display(powerData.temperature3)
        self.Current1613.display(powerData.cur16Pin1card3)
        self.Current1623.display(powerData.cur16Pin2card3)
        self.Current123.display(powerData.cur12card3)
        self.Voltage1613.display(powerData.volt16Pin1card3)
        self.Current1623.display(powerData.volt16Pin2card3)
        self.Current123.display(powerData.volt12card3)
        self.Battery3.display(powerData.battery3)
        self.Temperature4.display(powerData.temperature4)
        self.Current1614.display(powerData.cur16Pin1card4)
        self.Current1624.display(powerData.cur16Pin2card4)
        self.Current124.display(powerData.cur12card4)
        self.Voltage1614.display(powerData.volt16Pin1card4)
        self.Current1624.display(powerData.volt16Pin2card4)
        self.Current124.display(powerData.volt12card4)
        self.Battery4.display(powerData.battery4)


    def _handle_enableButton_16_1_clicked_Card1(self, checked):
        self.OutEnable1611.setEnabled(False)
        self.OutDisable1611.setEnabled(True)
        pass

    def _handle_disableButton_16_1_clicked_Card1(self, checked):
        self.OutEnable1611.setEnabled(True)
        self.OutDisable1611.setEnabled(False)
        pass

    def _handle_enableButton_16_2_clicked_Card1(self, checked):
        self.OutEnable1621.setEnabled(False)
        self.OutDisable1621.setEnabled(True)
        pass

    def _handle_disableButton_16_2_clicked_Card1(self, checked):
        self.OutEnable1621.setEnabled(True)
        self.OutDisable1621.setEnabled(False)
        pass

    def _handle_enableButton_12_clicked_Card1(self, checked):
        self.OutEnable121.setEnabled(False)
        self.OutDisable121.setEnabled(True)
        pass

    def _handle_disableButton_12_clicked_Card1(self, checked):
        self.OutEnable121.setEnabled(True)
        self.OutDisable121.setEnabled(False)
        pass
# --------------------------------------------------card2--------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
    def _handle_enableButton_16_1_clicked_Card2(self, checked):
        self.OutEnable1612.setEnabled(False)
        self.OutDisable1612.setEnabled(True)
        pass

    def _handle_disableButton_16_1_clicked_Card2(self, checked):
        self.OutEnable1612.setEnabled(True)
        self.OutDisable1612.setEnabled(False)
        pass

    def _handle_enableButton_16_2_clicked_Card2(self, checked):
        self.OutEnable1622.setEnabled(False)
        self.OutDisable1622.setEnabled(True)
        pass

    def _handle_disableButton_16_2_clicked_Card2(self, checked):
        self.OutEnable1622.setEnabled(True)
        self.OutDisable1622.setEnabled(False)
        pass

    def _handle_enableButton_12_clicked_Card2(self, checked):
        self.OutEnable122.setEnabled(False)
        self.OutDisable122.setEnabled(True)
        pass

    def _handle_disableButton_12_clicked_Card2(self, checked):
        self.OutEnable122.setEnabled(True)
        self.OutDisable122.setEnabled(False)
        pass
# ---------------------------------------------------card3-------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
    def _handle_enableButton_16_1_clicked_Card3(self, checked):
        self.OutEnable1613.setEnabled(False)
        self.OutDisable1613.setEnabled(True)
        pass

    def _handle_disableButton_16_1_clicked_Card3(self, checked):
        self.OutEnable1613.setEnabled(True)
        self.OutDisable1613.setEnabled(False)
        pass

    def _handle_enableButton_16_2_clicked_Card3(self, checked):
        self.OutEnable1623.setEnabled(False)
        self.OutDisable1623.setEnabled(True)
        pass

    def _handle_disableButton_16_2_clicked_Card3(self, checked):
        self.OutEnable1623.setEnabled(True)
        self.OutDisable1623.setEnabled(False)
        pass

    def _handle_enableButton_12_clicked_Card3(self, checked):
        self.OutEnable123.setEnabled(False)
        self.OutDisable123.setEnabled(True)
        pass

    def _handle_disableButton_12_clicked_Card3(self, checked):
        self.OutEnable123.setEnabled(True)
        self.OutDisable123.setEnabled(False)
        pass
# --------------------------------------------------card4--------------------------------------------------------------#
# ---------------------------------------------------------------------------------------------------------------------#
    def _handle_enableButton_16_1_clicked_Card4(self, checked):
        self.OutEnable1614.setEnabled(False)
        self.OutDisable1614.setEnabled(True)
        pass

    def _handle_disableButton_16_1_clicked_Card4(self, checked):
        self.OutEnable1614.setEnabled(True)
        self.OutDisable1614.setEnabled(False)
        pass

    def _handle_enableButton_16_2_clicked_Card4(self, checked):
        self.OutEnable1624.setEnabled(False)
        self.OutDisable1624.setEnabled(True)
        pass

    def _handle_disableButton_16_2_clicked_Card4(self, checked):
        self.OutEnable1624.setEnabled(True)
        self.OutDisable1624.setEnabled(False)
        pass

    def _handle_enableButton_12_clicked_Card4(self, checked):
        self.OutEnable124.setEnabled(False)
        self.OutDisable124.setEnabled(True)
        pass

    def _handle_disableButton_12_clicked_Card4(self, checked):
        self.OutEnable124.setEnabled(True)
        self.OutDisable124.setEnabled(False)
        pass
#----------------------------------------------------------------------------------------------------------------------#
#----------------------------------------------------------------------------------------------------------------------#

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



