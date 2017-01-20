import os
import rospy
import rospkg
import random
from Tkinter import Tk
from tkFileDialog import askopenfilename

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from sonia_msgs.srv import execute_cmd,get_information_list, manage_filterchain, copy_filterchain, save_filterchain


class ConfigWidget(QWidget):
    def __init__(self,main_window):
        super(ConfigWidget, self).__init__()
        try:
            rospy.wait_for_service('/provider_vision/get_information_list', timeout=2)
            rospy.wait_for_service('/provider_vision/execute_cmd', timeout=2)
            rospy.wait_for_service('/provider_vision/manage_filterchain', timeout=2)
            rospy.wait_for_service('/provider_vision/copy_filterchain', timeout=2)
            rospy.wait_for_service('/provider_vision/save_filterchain', timeout=2)
        except rospy.ROSException:
            rospy.loginfo('Services unavailable')
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_vision'), 'resource', 'config.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Configuration')

        self._current_filterchain = None
        self._current_media = None
        self._main_window = main_window

        ##### Service
        self._srv_get_information_list = rospy.ServiceProxy('/provider_vision/get_information_list', get_information_list)
        self._srv_execute_cmd = rospy.ServiceProxy('/provider_vision/execute_cmd', execute_cmd)
        self._srv_manage_filterchain = rospy.ServiceProxy('/provider_vision/manage_filterchain', manage_filterchain)
        self._srv_copy_filterchain = rospy.ServiceProxy('/provider_vision/copy_filterchain', copy_filterchain)
        self._srv_save_filterchain = rospy.ServiceProxy('/provider_vision/save_filterchain', save_filterchain)

        self.filterchain_list.currentIndexChanged[int].connect(self.current_filterchain_index_changed)
        self.filterchains.itemSelectionChanged.connect(self._handle_filterchains_selection_changed)
        self.media_list.currentIndexChanged[int].connect(self.current_media_index_changed)
        self.create_button.clicked[bool].connect(self._handle_create_new_execution)
        self.close_button.clicked[bool].connect(self.close)
        self.browseFile.clicked[bool].connect(self._handle_browse_file)
        self.copy_button.clicked[bool].connect(self._handle_copy_filterchain)
        self.delete_button.clicked[bool].connect(self._handle_delete_filterchain)
        self.add_button.clicked[bool].connect(self._handle_add_filterchain)
        self.name_text.setText('Execution_{}'.format(random.randint(1, 1000)))

        self.fill_filterchain_list()
        self.fill_media_list()

    def fill_filterchain_list(self):
        self._current_filterchain = None
        self.filterchain_list.clear()
        self.filterchains.clear()

        try:
            filterchain_string = self._srv_get_information_list(3)
        except rospy.ServiceException as err:
            rospy.logerr(err)

        filterchain_list = filterchain_string.list.split(';')
        if len(filterchain_list) == 0:
            return
        for filterchain in filterchain_list:
            if len(filterchain) > 0:
                self.filterchain_list.addItem(filterchain)
                self.filterchains.addItem(filterchain)

        self.delete_button.setEnabled(False)
        self.copy_button.setEnabled(False)

    def fill_media_list(self):
        self._current_media = None
        self.media_list.clear()

        try:
            media_string = self._srv_get_information_list(2)
        except rospy.ServiceException as err:
            rospy.logerr(err)

        media_list = media_string.list.split(';')
        if len(media_list) == 0:
            return
        for media in media_list:
            if len(media) > 0:
                self.media_list.addItem(media)

    def current_filterchain_index_changed(self, index):
        filterchain = self.filterchain_list.itemText(index)
        self._current_filterchain = filterchain

    def current_media_index_changed(self, index):
        media = self.media_list.itemText(index)
        self._current_media = media

    def _handle_create_new_execution(self):
        if len(self.name_text.text()) == 0:
            return;
        media = self.video_text.text()
        if len(media) == 0:
            media = self._current_media
        try:
            self._srv_execute_cmd(self.name_text.text(), self._current_filterchain,media, 1)
        except rospy.ServiceException as err:
            rospy.logerr(err)

        self._main_window.fill_execution_list()
        self._main_window.current_execution.setCurrentIndex(self._main_window.current_execution.count() -1)
        self.close()

    def _handle_browse_file(self):
        Tk().withdraw()
        filename = askopenfilename()
        self.video_text.setText(filename)

    def _handle_filterchains_selection_changed(self):
        if self.filterchains.currentRow() >= 0:
            self._current_selected_filterchain = self.filterchains.currentItem().text()
            self.delete_button.setEnabled(True)
            self.copy_button.setEnabled(True)
        else :
            self.delete_button.setEnabled(False)
            self.copy_button.setEnabled(False)

    def _handle_copy_filterchain(self):
        if len(self.filterchain_name.text()) == 0:
            return;
        try:
            self._srv_copy_filterchain(self._current_selected_filterchain, 'filterchain/' + self.filterchain_name.text())
            self.fill_filterchain_list()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _handle_delete_filterchain(self):
        try:
            self._srv_manage_filterchain('filterchain/' + self._current_selected_filterchain, 2)
            self.fill_filterchain_list()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _handle_add_filterchain(self):
        if len(self.filterchain_name.text()) == 0:
            return;

        try:
            self._srv_manage_filterchain(self.filterchain_name.text(), 1)
            self.fill_filterchain_list()
        except rospy.ServiceException as err:
            rospy.logerr(err)
