import os
import rospy
import rospkg

from datetime import datetime

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QTableWidgetItem


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

        freq_range = range(freq_value - 1, freq_value + 1)

        rowPosition = self.table_hydro_ping_data.rowCount()
        self.table_hydro_ping_data.insertRow(rowPosition)

        time = "{}:{}".format(msg.header.stamp.secs, msg.header.stamp.nsecs)

        time_item = QTableWidgetItem(time)
        sequence_item = QTableWidgetItem(str(msg.header.seq))
        noise_item = QTableWidgetItem(str(msg.noise))
        heading_item = QTableWidgetItem(str(msg.heading))
        amplitude_item = QTableWidgetItem(str(msg.amplitude))
        elevation_item = QTableWidgetItem(str(msg.elevation))

        self.table_hydro_ping_data.setItem(rowPosition, 0, time_item)
        self.table_hydro_ping_data.setItem(rowPosition, 1, sequence_item)
        self.table_hydro_ping_data.setItem(rowPosition, 2, noise_item)
        self.table_hydro_ping_data.setItem(rowPosition, 3, heading_item)
        self.table_hydro_ping_data.setItem(rowPosition, 4, amplitude_item)
        self.table_hydro_ping_data.setItem(rowPosition, 5, elevation_item)

        self.table_hydro_ping_data.resizeRowsToContents()

        print(freq_range)
        #if freq_value in freq_range:
         #   time = "{}:{}".format(msg.header.stamp.secs, msg.header.stamp.nsecs)
         #   ls = [time, msg.header.seq, msg.noise, msg.heading,  msg.amplitude, msg.elevation]
            #print(ls)
        #self.table_hydro_ping_data.addItems(ls)


    def shutdown_plugin(self):
        self._provider_hydro_subscriber.unregister()
