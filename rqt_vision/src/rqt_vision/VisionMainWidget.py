import os
import rospy
import rospkg
import cv2
import threading
import time

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QPainter, QImage
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QMenu,QAction,QWidget
from sensor_msgs.msg import Image as SensorImage
from sonia_msgs.msg import VisionTarget
from sonia_msgs.srv import get_filterchain_from_execution, get_media_from_execution, execute_cmd
from cv_bridge import CvBridge, CvBridgeError
from ConfigureFilterchainWidget import ConfigureFilterchainWidget
from Tkinter import Tk
from tkFileDialog import asksaveasfilename

from sonia_msgs.srv import get_information_list


class VisionMainWidget(QWidget):
    result_change = pyqtSignal('QString')
    def __init__(self):
        super(VisionMainWidget, self).__init__()
        try:
            rospy.wait_for_service('/provider_vision/get_information_list', timeout=2)
            rospy.wait_for_service('/provider_vision/get_filterchain_from_execution', timeout=2)
            rospy.wait_for_service('/provider_vision/get_media_from_execution', timeout=2)
            rospy.wait_for_service('/provider_vision/execute_cmd', timeout=2)
        except rospy.ROSException:
            rospy.loginfo('Services unavailable')
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_vision'), 'resource', 'mainwidget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Vision UI')

        self._current_execution = None
        self._current_execution_subscriber = None
        self._current_execution_subscriber_result = None
        self._is_recording = False
        self._video_writer = None
        self._cv_image = None
        self._widget = None
        self.bridge = CvBridge()
        self.result_change.connect(self._handle_result)
        ##### Service
        self._srv_get_information_list = rospy.ServiceProxy('/provider_vision/get_information_list', get_information_list)
        self._srv_get_filterchain_from_execution = rospy.ServiceProxy('/provider_vision/get_filterchain_from_execution', get_filterchain_from_execution)
        self._srv_get_media_from_execution = rospy.ServiceProxy('/provider_vision/get_media_from_execution', get_media_from_execution)
        self._srv_execute_cmd = rospy.ServiceProxy('/provider_vision/execute_cmd', execute_cmd)

        ###
        self.image_frame_mouse_release_event_original = self.imageFrame.mouseReleaseEvent
        self.imageFrame.mouseReleaseEvent = self.image_frame_mouse_release_event

        self.refresh_button.clicked[bool].connect(self.fill_execution_list)
        self.current_execution.currentIndexChanged[int].connect(self.current_execution_index_changed)
        self.image_paint_event_original = self.imageFrame.paintEvent
        self.imageFrame.paintEvent = self.image_paint_event
        self.fill_execution_list()
        self._define_menu()

    def _define_menu(self):
        self._menu = QMenu(self.imageFrame)

        configureAction = QAction("Configure ...", self.imageFrame, triggered=self.configure_filterchain_action)
        self._menu.addAction(configureAction)
        self._menu.addSeparator()
        self._recordAction = QAction("Record ...", self.imageFrame, triggered=self.record_execution_action)
        self._menu.addAction(self._recordAction)
        self._stop_recordAction = QAction("Stop Record", self.imageFrame, triggered=self.stop_record_execution_action)
        self._stop_recordAction.setEnabled(False)
        self._menu.addAction(self._stop_recordAction)
        self._menu.addSeparator()

        deleteAction = QAction(self.imageFrame.tr("Delete this execution"), self.imageFrame, triggered=self.delete_current_execution)
        self._menu.addAction(deleteAction)


    def fill_execution_list(self):
        self._refresh_clean()
        self._current_execution = None
        self.current_execution.clear()

        execution_string = self._srv_get_information_list(1)
        execution_list = execution_string.list.split(';')
        if len(execution_list) == 0:
            return
        self._current_execution = execution_list[0]
        for execution in execution_list:
            if len(execution) > 0:
                self.current_execution.addItem(execution)

    def current_execution_index_changed(self, index):
        self._refresh_clean()

        if self._current_execution_subscriber is not None:
            self._current_execution_subscriber.unregister()
            self._current_execution_subscriber_result.unregister()

        new_execution = self.current_execution.itemText(index)
        self._current_execution = new_execution
        self._filterchain = self._srv_get_filterchain_from_execution(self._current_execution)
        self._current_execution_subscriber = rospy.Subscriber('/provider_vision/' + new_execution + '_image',
                                                              SensorImage, self.current_execution_callback)
        self._current_execution_subscriber_result = rospy.Subscriber('/provider_vision/' + new_execution + '_result',
                                                                     VisionTarget,
                                                                     self.current_execution_result_callback)


    def _refresh_clean(self):
        if self._current_execution_subscriber is not None:
            self._current_execution_subscriber.unregister()
            self._current_execution_subscriber_result.unregister()
        self.result_text.setText('')
        self._cv_image = None
        self.imageFrame.update()

    def current_execution_callback(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="rgb8")
            height, width, channel = cv_image.shape
            bytesPerLine = 3 * width
            self._cv_image = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)

            if self._is_recording:
                if self._video_writer is None:
                    four_cc = cv2.cv.CV_FOURCC(*'HFYU')
                    self._video_writer = cv2.VideoWriter(self._recording_file_name, four_cc, float(15), (width,height))
                temp = cv2.cvtColor(cv_image,cv2.cv.CV_BGR2RGB)
                self._video_writer.write(temp)
        except CvBridgeError as e:
            print(e)
        self.imageFrame.update()

    def image_paint_event(self, data):
        self.image_paint_event_original(data)
        if self._cv_image is None:
            return;
        img = self._cv_image

        painter = QPainter(self.imageFrame)
        painter.drawImage(data.rect(), img)
        painter.end()

    def current_execution_result_callback(self, visionTarget):
        result = 'X:{}, Y:{}, width:{:.2f}, height:{:.2f}, angle:{:.2f}, desc_1:{}, desc_2:{}'.format(visionTarget.x,
                                                                                                      visionTarget.y,
                                                                                                      visionTarget.width,
                                                                                                      visionTarget.height,
                                                                                                      visionTarget.angle,
                                                                                                      visionTarget.desc_1,
                                                                                                      visionTarget.desc_2)
        self.result_change.emit(result)

    def _handle_result(self,result):
        self.result_text.setText(result)

    def image_frame_mouse_release_event(self,event):
        if event.button() == Qt.RightButton:
            self._menu.exec_(self.imageFrame.mapToGlobal(event.pos()))

    def delete_current_execution(self):

        if self._current_execution is None :
            return

        media = self._srv_get_media_from_execution(self._current_execution)

        self._srv_execute_cmd(self._current_execution,self._filterchain.list,media.list,2)

        self.fill_execution_list()

    def configure_filterchain_action(self):
        self._widget = ConfigureFilterchainWidget(self._current_execution, self._filterchain)
        self._widget.show()

    def shutdown_plugin(self):
        if self._current_execution_subscriber is not None:
            self._current_execution_subscriber.unregister()
            self._current_execution_subscriber_result.unregister()
        if self._widget is not None:
            self._widget.close()

    def record_execution_action(self):
        Tk().withdraw()
        filename = asksaveasfilename(defaultextension='.avi')
        self._recording_file_name = filename
        if len(self._recording_file_name) > 0:
            self._is_recording = True
            self._recordAction.setEnabled(False)
            self._stop_recordAction.setEnabled(True)

    def stop_record_execution_action(self):
        self._is_recording = False
        time.sleep(0.1)
        self._video_writer.release()
        self._video_writer = None
        self._recordAction.setEnabled(True)
        self._stop_recordAction.setEnabled(False)







