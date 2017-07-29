from __future__ import division
import os
import rospkg
from threading import Thread
import rospy
import time
import math
from proc_image_processing.msg import VisionTarget
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal
from proc_hydrophone.msg import PingPose


# main class inherits from the ui window class
class ManualHydroWidget(QWidget):

    def __init__(self):
        super(ManualHydroWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_manual_hydro'), 'resource', 'mainWidget.ui')
        loadUi(ui_file, self)
        self.topic_ping = None
        self.thread = None
        self.StartBtn.clicked.connect(self._handle_start)
        self.StopBtn.clicked.connect(self._handle_stop)


    def _handle_start(self):
        try:
            if self.topic_ping is not None :
                self.topic_ping.unregister()
            if self.thread is not None:
                self.thread_running = False
                self.thread.join()
            self.thread_running = True
            self.topic_ping = rospy.Publisher('/proc_hydrophone/ping', PingPose, queue_size=10)
            self.thread = Thread(target=self._publish_in_continuous)
            self.thread.start()
        except rospy.ROSException, e:
            rospy.logerr('unable to publish')

    def _handle_stop(self):
        self.thread_running = False

    def _publish_in_continuous(self):
        while not rospy.is_shutdown() and self.thread_running:
            print 'publishing hydro'
            target = PingPose()
            target.pose.orientation.z = math.radians(self.heading_spin.value() %360)

            self.topic_ping.publish(target)
            rospy.sleep(self.thread_duration.value())

    def shutdown_plugin(self):
        if self.topic_ping is not None:
            self.topic_ping.unregister()
        pass