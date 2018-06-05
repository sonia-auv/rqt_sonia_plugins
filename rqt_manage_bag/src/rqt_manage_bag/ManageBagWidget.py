import os
import rospkg
import rospy
import rosbag
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from Tkinter import Tk
from tkFileDialog import askopenfilename
from os.path import expanduser
from sensor_msgs.msg import CompressedImage


class ManageBagWidget(QMainWindow):

    def __init__(self):
        super(ManageBagWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('ManageBagWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_manage_bag'), 'resource', 'mainwindow.ui')
        loadUi(ui_file, self)

        if not os.path.exists(expanduser("~/Bags")):
            os.makedirs(expanduser("~/Bags"))

        self.bag_file = None
        self.ros_bag = None
        self.topic_name = None
        self.new_ros_bag = None
        self.start_time = None
        self.stop_time = None
        self.check_time = False
        self.start = False
        self.stop = False
        self.frame = 1
        self.i = 0

        self.actionLoad_bag_File.triggered.connect(self._handle_load)
        self.StartpushButton.clicked.connect(self._handle_start)
        self.StoppushButton.clicked.connect(self._handle_stop)
        self.RunpushButton.clicked.connect(self._handle_run)
        self.FPS.valueChanged.connect(self._value_change)

        self.StartpushButton.setEnabled(False)
        self.StoppushButton.setEnabled(False)

    def _value_change(self):
        self.frame = self.FPS.value()

    def _handle_load(self):
        # instantiate a Tk window
        root = Tk()
        root.withdraw()
        self.bag_file = askopenfilename(defaultextension='.bag', initialdir=expanduser("~/Bags"))
        if self.bag_file:
            self.BagName.setText(self.bag_file)
            self.ros_bag = rosbag.Bag(self.bag_file)

    def _handle_start(self):
        self.StartpushButton.setEnabled(False)
        self.StoppushButton.setEnabled(True)
        self.start = True
        self.stop = False

    def _handle_stop(self):
        self.StartpushButton.setEnabled(True)
        self.StoppushButton.setEnabled(False)
        self.start = False
        self.stop = True

    def _handle_run(self):
        self.StartpushButton.setEnabled(True)
        self.RunpushButton.setEnabled(False)
        self.topic_name = self.TopicName.text()
        self.new_ros_bag = self.NewBagName.text()
        self.bag_file = self.BagName.text()
        self.start_thread()

    def start_thread(self):
        t1 = threading.Thread(target=self.manage_bag)
        t1.setDaemon(1)
        t1.start()

    def manage_bag(self):
        try:
            outbag = rosbag.Bag(expanduser("~/Bags") + '/' + self.new_ros_bag, 'w')
            pub = rospy.Publisher(self.topic_name, CompressedImage, queue_size=100)
            for topic, msg, t in self.ros_bag.read_messages():
                rate = rospy.Rate(self.frame)
                start, stop = self.start, self.stop
                if topic == self.topic_name:
                    pub.publish(msg)

                    if start and not stop:
                        outbag.write(topic, msg)
                rate.sleep()

            outbag.close()

            outbag.reindex()

            self.i += 1
            self.StartpushButton.setEnabled(False)
            self.StoppushButton.setEnabled(False)
            self.RunpushButton.setEnabled(True)

            rospy.loginfo('The new bag is create %i' % self.i)
        except:
            self.StartpushButton.setEnabled(False)
            self.RunpushButton.setEnabled(True)
            rospy.logerr('Sorry something wrong')

    def _handle_start_test_triggered(self):
        pass

    def _execute_test(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
