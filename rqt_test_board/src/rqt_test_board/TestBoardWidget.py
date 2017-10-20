from __future__ import division
import os
import rospkg
from threading import Thread
import rospy
import time
from interface_rs485.msg import SendRS485Msg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtCore import pyqtSignal


# main class inherits from the ui window class
class TestBoardWidget(QMainWindow):

    def __init__(self):
        super(TestBoardWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_test_board'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)
        self.topic_result = rospy.Publisher('/interface_rs485/dataRx', SendRS485Msg, queue_size=100)
        self.thread = None
        self.Start.clicked.connect(self._handle_start)
        self.Stop.clicked.connect(self._handle_stop)

    def _handle_start(self):
        try:
            self.slave = int(self.Slave.text())
            self.cmd = int(self.Cmd.text())
            self.data = list([int(self.Data.text())])
            self.rate = rospy.Rate(int(self.Rate.text()))
            self.thread_running = True
            self.thread = Thread(target=self._publish_in_continuous)
            self.thread.start()
        except ValueError:
            rospy.logerr('unable to publish')

    def _handle_stop(self):
        self.thread_running = False

    def _publish_in_continuous(self):
        while not rospy.is_shutdown() and self.thread_running:
            print 'publishing'
            msg = SendRS485Msg()
            msg.slave = self.slave
            msg.cmd = self.cmd
            msg.data = self.data
            self.topic_result.publish(msg)
            self.rate.sleep()

    def shutdown_plugin(self):
        pass
