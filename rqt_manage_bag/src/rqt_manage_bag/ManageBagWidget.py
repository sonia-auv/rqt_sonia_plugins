import os
import rospkg
import rospy
import rosbag

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from Tkinter import Tk
from tkFileDialog import askopenfilename
from os.path import expanduser


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

        self.actionLoad_bag_File.triggered.connect(self._handle_load)
        self.StartpushButton.clicked.connect(self._handle_start)

    def _handle_load(self):
        # instantiate a Tk window
        root = Tk()
        root.withdraw()
        self.bag_file = askopenfilename(defaultextension='.bag', initialdir=expanduser("~/Bags"))
        if self.bag_file:
            self.BagName.setText(self.bag_file)
            self.ros_bag = rosbag.Bag(self.bag_file)

    def _handle_start(self):
        self.topic_name = self.TopicName.text()
        self.new_ros_bag = self.NewBagName.text()
        self.bag_file = self.BagName.text()
        if self.CheckTime.checkState():
            self.check_time = True
            self.start_time = int(self.StartTime.text())
            self.stop_time = int(self.StopTime.text())
        self.manage_bag()

    def manage_bag(self):
        time = None
        outbag = rosbag.Bag(expanduser("~/Bags") + '/' + self.new_ros_bag, 'w')

        for topic, msg, t in self.ros_bag.read_messages():
            if time is None:
                time = t

            new_time = t
            time_now = float((new_time - time).secs)

            if topic == self.topic_name:
                if not self.check_time:
                    outbag.write(topic, msg, t)
                elif time_now >= self.start_time and time_now <= self.stop_time:
                    outbag.write(topic, msg, t)

        outbag.close()

        outbag.reindex()

        rospy.loginfo('The new bag is create')

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


