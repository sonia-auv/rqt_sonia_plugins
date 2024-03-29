import os
import rospkg
import rospy
import rosbag
import threading
import uuid
import cv2

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QFileDialog, QMessageBox
from os.path import expanduser
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

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
        self.out_folder = None
        self.ros_bag = None
        self.topic_name = ' '
        self.new_ros_bag = None
        self.start_time = None
        self.stop_time = None
        self.check_time = False
        self.start = False
        self.stop = False
        self.frame = 10
        self.i = 0
        self.j = 0
        self.outbag = None
        self.in_extract = None
        self.out_extract = None
        self.value = 0
        self.lcdNumberFPS.display(self.frame)

        self.selectInputBag.clicked.connect(self._handle_load)
        self.selectOutputFolder.clicked.connect(self._handle_select_folder)
        self.startStopPushButton.clicked.connect(self._handle_start)
        self.runPushButton.clicked.connect(self._handle_run)
        self.FPS.valueChanged.connect(self._value_change)
        self.topicName.currentTextChanged.connect(self._handle_topic_name_change)
        self.saveBagButton.clicked.connect(self._save_bag)
        self.inputFolderExtract.clicked.connect(self._input_folder_extract)
        self.outFolderExtract.clicked.connect(self._out_folder_extract)
        self.extracButton.clicked.connect(self._start_extract_image)
        self.resetButton.clicked.connect(self._reset_segmentation)

        self.startStopPushButton.setEnabled(False)
        self.runPushButton.setEnabled(False)
        self.saveBagButton.setEnabled(False)
        self.extracButton.setEnabled(False)
        self.saveBagButton.setEnabled(False)
        self.selectOutputFolder.setEnabled(False)
        self.resetButton.setEnabled(False)

    def _reset_segmentation(self):
        self.startStopPushButton.setEnabled(False)
        self.runPushButton.setEnabled(False)
        self.saveBagButton.setEnabled(False)
        self.extracButton.setEnabled(False)
        self.saveBagButton.setEnabled(False)
        self.selectOutputFolder.setEnabled(False)
        self.resetButton.setEnabled(False)

        self.label_newBagName.setText("")
        self.inputBag.setText("")
        self.outputFolder.setText("")
        self.nameObject.setText("")
        self.locationObject.setText("")
        self.dateObject.setText("")

    def _get_file_name_list(self, path):
        dir_list = os.listdir(path)
        bag_path_list = []
        for directory in dir_list:
            source_bag_dir = os.path.join(path, directory)
            file_list = os.listdir(source_bag_dir)
            for file_name in file_list:
                bag_path = directory, os.path.join(source_bag_dir, file_name)
                bag_path_list.append(bag_path)
        return bag_path_list

    def _start_extract_image(self):
        self.start_thread(self._extract_image)

    def _extract_image(self):
        self.value = 0
        bag_path_list = self._get_file_name_list(self.in_extract)
        bridge = CvBridge()

        for directory, bag_path in bag_path_list:
            out_path = os.path.join(self.out_extract, directory)
            if not os.path.exists(out_path):
                os.makedirs(out_path)
            i = 0
            with rosbag.Bag(bag_path, "r") as bag:
                for topic, msg, _ in bag.read_messages():
                    if i >= 6:
                        img_name = "frame_{}.jpg".format(str(uuid.uuid1()))
                        extraction_path = os.path.join(out_path, img_name)
                        self.value += 0.01
                        if self.value >= 99:
                            self.value = 99
                        self.progressBar.setValue(self.value)
                        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                        cv2.imwrite(extraction_path, cv_img)
                        i = 0
                    i += 1
        self.progressBar.setValue(100)

    def _input_folder_extract(self):
        self.in_extract = self._select_folder("Select Input Directory")
        self.inputNameExtract.setText(self.in_extract)

    def _out_folder_extract(self):
        self.out_extract = self._select_folder("Select Output Directory")
        self.outNameExtract.setText(self.out_extract)
        self.extracButton.setEnabled(True)

    def _select_folder(self, msg):
        return str(QFileDialog.getExistingDirectory(self, msg))

    # Bouton saveBagButton

    def _save_bag(self):
        out_directory = os.path.join(self.outputFolder.text(), self.nameObject.text())
        if not os.path.exists(out_directory):
            os.makedirs(out_directory)
        try:
            # bag_name = "{}.bag".format(str(uuid.uuid1()))
            for filename in os.listdir(out_directory):
                if filename.endswith(".bag"):
                    self.j += 1
            if 'front' in self.topic_name or 'Front' in self.topic_name:
                self.camera = "front"
            else:
                self.camera = "bottom"

            bag_name = self.camera + "_" + self.nameObject.text() + "_" + self.locationObject.text() + "_" + self.dateObject.text() + "_" + str(self.j) + ".bag"
            self.label_newBagName.setText(bag_name)
            os.rename(os.path.join(expanduser("~/Bags"), self.new_ros_bag), os.path.join(out_directory, bag_name))
            rospy.loginfo('The new bag %s is located in %s' % (bag_name, out_directory))
            self.resetButton.setEnabled(True)
        except:
            rospy.loginfo("Error in save bag")
            self._message_box("No object selected")

    # ComboBox topicName

    def _handle_topic_name_change(self):
        self.topic_name = self.topicName.currentText()

    # QSlide FPS

    def _value_change(self):
        self.frame = self.FPS.value()
        self.lcdNumberFPS.display(self.frame)

    # Bouton selectOutputFolder

    def _handle_select_folder(self):
        self.out_folder = self._select_folder("Select Output Directory")
        self.outputFolder.setText(self.out_folder)
        self.saveBagButton.setEnabled(True)

    # Bouton selectInputBag

    def _handle_load(self):
        self.bag_file = QFileDialog.getOpenFileName(self, caption="Select Input Bag", directory=expanduser("~/Bags"))[0]
        if self.bag_file[-4:] == ".bag":
            self.inputBag.setText(self.bag_file)
            self.ros_bag = rosbag.Bag(self.bag_file)
            self.runPushButton.setEnabled(True)
            topics = self.ros_bag.get_type_and_topic_info()[1].keys()
            for i in range(0,len(topics)):
                self.topicName.addItem(list(topics)[i])

    # Bouton startStopPushButton

    def _handle_start(self):
        if not self.start:
            self.startStopPushButton.setText("Stop recording")
            self.start = True
            self.stop = False
        else:
            self.startStopPushButton.setText("Start recording")
            self.start = False
            self.stop = True

    # Bouton runPushButton

    def _handle_run(self):
        self.startStopPushButton.setEnabled(True)
        self.runPushButton.setEnabled(False)
        self.selectOutputFolder.setEnabled(False)
        self.saveBagButton.setEnabled(False)
        self.new_ros_bag = str(uuid.uuid1()) + '.bag'
        self.start_thread(self.manage_bag)
        self.label_newBagName.setText(self.new_ros_bag)

    def start_thread(self, target):
        t1 = threading.Thread(target=target)
        t1.setDaemon(1)
        t1.start()

    def manage_bag(self):
        try:
            self.outbag = rosbag.Bag(os.path.join(expanduser("~/Bags"), self.new_ros_bag), 'w')
            pub = rospy.Publisher(self.topic_name, CompressedImage, queue_size=100)
            for topic, msg, t in self.ros_bag.read_messages():
                rate = rospy.Rate(self.frame)
                start, stop = self.start, self.stop
                if topic == self.topic_name:
                    pub.publish(msg)

                    if start and not stop:
                        self.outbag.write(topic, msg)
                rate.sleep()

            self.outbag.close()

            self.outbag.reindex()

            self.i += 1
            self.startStopPushButton.setEnabled(False)
            self.runPushButton.setEnabled(True)
            self.selectOutputFolder.setEnabled(True)

            rospy.loginfo('The new bag is created, ready to be saved')
        except:
            self.startStopPushButton.setEnabled(False)
            self.runPushButton.setEnabled(True)
            rospy.loginfo("Something has gone wrong in manage_bag")

    def _message_box(self, msg):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setText(msg)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

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
