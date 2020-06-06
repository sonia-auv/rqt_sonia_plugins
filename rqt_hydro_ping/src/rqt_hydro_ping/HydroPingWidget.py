import os
import rospy
import rospkg

from datetime import datetime

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QTableWidgetItem


from sonia_msgs.msg import PingMsg


class HydroPingWidget(QWidget):
    PING_LIST_MAX_ITEM_COUNT = 250
    _monitor_hydro_ping_msg = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(HydroPingWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'rqt_hydro_ping'), 'resource', 'mainwindow.ui')

        loadUi(ui_file, self)

        self.ping_list = []

        self._provider_hydro_subscriber = rospy.Subscriber('/provider_hydrophone/ping', PingMsg,
                                                           self._handle_provider_hydro_ping_msg)

        self.slider_hydro_freq.valueChanged.connect(
            self._handle_hydro_frequency_slider_change)

        self._monitor_hydro_ping_msg.connect(
            self._received_provider_hydro_ping_msg)

    def _handle_provider_hydro_ping_msg(self, msg):
        self._monitor_hydro_ping_msg.emit(msg)

    def _handle_hydro_frequency_slider_change(self):
        freq_value = self.slider_hydro_freq.value()
        self.lbl_hydro_freq_value.setText('{} Hz'.format(freq_value))

    def _received_provider_hydro_ping_msg(self, msg):
        self.save_ping_data(msg)

    def save_ping_data(self, provider_hydro_ping_data):
        if len(self.ping_list) == self.PING_LIST_MAX_ITEM_COUNT:
            self.ping_list.pop(0)

        self.ping_list.append(provider_hydro_ping_data)

        self.handle_table_content()

    def handle_table_content(self):

        freq_value = self.slider_hydro_freq.value()

        freq_range = range(freq_value - 1, freq_value + 1)

        self.table_hydro_ping_data.setRowCount(0)

        for ping_item in self.ping_list:
            if ping_item.frequency in freq_range:
                self.insert_ping_data_row(ping_item)

    def insert_ping_data_row(self, provider_hydro_ping_data):

        time = "{}:{}".format(
            provider_hydro_ping_data.header.stamp.secs,
            provider_hydro_ping_data.header.stamp.nsecs)

        time_item = QTableWidgetItem(time)
        sequence_item = QTableWidgetItem(
            str(provider_hydro_ping_data.header.seq))
        frequency_item = QTableWidgetItem(
            str(provider_hydro_ping_data.frequency))
        noise_item = QTableWidgetItem(str(provider_hydro_ping_data.noise))
        heading_item = QTableWidgetItem(str(provider_hydro_ping_data.heading))
        amplitude_item = QTableWidgetItem(
            str(provider_hydro_ping_data.amplitude))
        elevation_item = QTableWidgetItem(
            str(provider_hydro_ping_data.elevation))

        row_position = self.table_hydro_ping_data.rowCount()

        self.table_hydro_ping_data.setRowHeight(row_position - 1, 33)
        self.table_hydro_ping_data.insertRow(row_position)

        self.table_hydro_ping_data.setItem(row_position, 0, time_item)
        self.table_hydro_ping_data.setItem(row_position, 1, frequency_item)
        self.table_hydro_ping_data.setItem(row_position, 2, sequence_item)
        self.table_hydro_ping_data.setItem(row_position, 3, noise_item)
        self.table_hydro_ping_data.setItem(row_position, 4, heading_item)
        self.table_hydro_ping_data.setItem(row_position, 5, amplitude_item)
        self.table_hydro_ping_data.setItem(row_position, 6, elevation_item)

        self.table_hydro_ping_data.resizeRowsToContents()

    def shutdown_plugin(self):
        self._provider_hydro_subscriber.unregister()
