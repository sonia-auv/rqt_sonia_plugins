from __future__ import division
import os
import rospkg
from threading import Thread
import rospy
import time
from sonia_msgs.msg import VisionTarget
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import pyqtSignal


# main class inherits from the ui window class
class ManualVisionWidget(QWidget):

    def __init__(self):
        super(ManualVisionWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_manual_vision'), 'resource', 'mainWidget.ui')
        loadUi(ui_file, self)
        self.topic_result = None
        self.thread = None
        self.StartBtn.clicked.connect(self._handle_start)
        self.StopBtn.clicked.connect(self._handle_stop)


    def _handle_start(self):
        try:
            if self.topic_result is not None :
                self.topic_result.unregister()
            if self.thread is not None:
                self.thread_running = False
                self.thread.join()
            self.thread_running = True
            self.topic_result = rospy.Publisher(self.topic_name.text(), VisionTarget, queue_size=10)
            self.thread = Thread(target=self._publish_in_continuous)
            self.thread.start()
        except rospy.ROSException, e:
            rospy.logerr('unable to publish')

    def _handle_stop(self):
        self.thread_running = False

    def _publish_in_continuous(self):
        while not rospy.is_shutdown() and self.thread_running:
            print 'publishing'
            target = VisionTarget()
            target.x = self.x_spin.value()
            target.y = self.y_spin.value()
            target.width = self.width_spin.value()
            target.height = self.height_spin.value()
            target.angle = self.angle_spin.value()
            target.desc_1 = self.spec_text.text()
            target.desc_2 = self.spec_text.text()

            self.topic_result.publish(target)
            rospy.sleep(self.thread_duration.value())

    def shutdown_plugin(self):
        if self.topic_result is not None:
            self.topic_result.unregister()
