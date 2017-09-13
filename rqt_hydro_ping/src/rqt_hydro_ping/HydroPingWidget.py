import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal

from provider_hydrophone.msg import PingMsg


class HydroPingWidget(QWidget):
    _monitor_hydro_ping_msg = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(HydroPingWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_hydro_ping'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)

        self.setWindowTitle('Hydro Ping')

        self._provider_hydro_subscriber = rospy.Subscriber('/provider_hydrophone/provider_hydrophone', PingMsg,
                                                          self._handle_provider_hydro_ping_msg)

        self.slider_hydro_freq.valueChanged.connect(self._handle_hydro_frequency_slider_change)

        self._monitor_hydro_ping_msg.connect(self._received_provider_hydro_ping_msg)


    def _handle_provider_hydro_ping_msg(self, msg):
        self._monitor_hydro_ping_msg.emit(msg)

    def _handle_hydro_frequency_slider_change(self):
        freq_value = self.slider_hydro_freq.value()
        self.lbl_hydro_freq_value.setText('{} Hz'.format(freq_value))

    def _received_provider_hydro_ping_msg(self, msg):
        freq_value = self.slider_hydro_freq.value()

        freq_value_min = freq_value - 1
        freq_value_max = freq_value + 1

        if freq_value_min >= freq_value <= freq_value_max:
            self.table_hydro_ping_data.addItem(msg)


    def shutdown_plugin(self):
        self._provider_hydro_subscriber.unregister()
