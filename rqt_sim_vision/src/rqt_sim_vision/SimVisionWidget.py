import os
import rospkg
import rospy
import math

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget
from python_qt_binding.QtCore import pyqtSignal, QPointF
from sonia_common.msg import VisionTarget
from nav_msgs.msg import Odometry
from .Renderer import Buoy, Path


class SimVisionWidget(QMainWindow):
    odom_result_data_buoy = pyqtSignal(Odometry)
    odom_result_data_fence = pyqtSignal(Odometry)
    odom_result_data_path_finder = pyqtSignal(Odometry)

    ellipse_width = 80
    ellipse_height = 80

    rect_width = 50
    rect_height = 100
    angle = 30

    buoy_diameter = 0.23

    path_width = 0.15
    path_height = 1.2

    def __init__(self):
        super(SimVisionWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('SimVisionWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_sim_vision'), 'resource', 'mainwindow.ui')

        loadUi(ui_file, self)

        self.odom_result_data_buoy.connect(self.show_buoys_position)
        self.odom_result_data_fence.connect(self.show_fence_position)
        self.odom_result_data_path_finder.connect(self.show_path_finder_position)

        self.mousePressEvent = self.my_mouse_press_event

        self.displayed = 0

        self.position_x = 0
        self.position_y = 0

        self.last_position_x = 0
        self.last_position_y = 0
        self.last_position_z = 0
        self.last_yaw = 0

        self.posx_sub = -20
        self.posy_sub = -20

        self.scale_value = 0

        self.Buoy = False
        self.Fence = False
        self.Path_fender = False

        self._odom_subscriber = rospy.Subscriber('/telemetry/auv_states', Odometry, self._odom_callback)
        #self._odom_subscriber = rospy.Subscriber('/proc_nav/auv_states', Odometry, self._odom_callback)

        self._publish_image_data = rospy.Publisher('/proc_image_processing/result', VisionTarget, queue_size=10)

        self.setObjectName('MySimVisionWidget')

        self.actionBuoy.triggered.connect(self._handle_buoy)
        self.actionFence.triggered.connect(self._handle_fence)
        self.actionPath_finder.triggered.connect(self._handle_path_finder)

        self.object_type = Buoy(self)
        self.object_width = 0
        self.object_height = 0

    def _odom_callback(self, data):
        if self.Buoy:
            self.odom_result_data_buoy.emit(data)
        elif self.Fence:
            self.odom_result_data_fence.emit(data)
        elif self.Path_fender:
            self.odom_result_data_path_finder.emit(data)

    def my_mouse_press_event(self, mouseEvent):
        self.displayed = 1
        self.position_x = mouseEvent.x()
        self.position_y = mouseEvent.y()
        self.posx_sub = self.position_x
        self.posy_sub = self.position_y
        self.update()

    def paintEvent(self, event):

        sub_x = self.posx_sub
        sub_y = self.posy_sub
        angle = self.angle

        if self.Buoy:
            self.object_type.paint_object(sub_x - self.object_width / 2, sub_y - self.object_height / 2, self.object_width, self.object_height, angle)
        if self.Path_fender:
            self.object_type.paint_object(sub_x, sub_y, self.object_width, self.object_height, angle)

    def set_data(self, pos_x, pos_y, width, height, angle=0):
        data = VisionTarget()

        data.header = 'simulation'
        data.x = pos_x - 320
        data.y = pos_y - 240
        data.width = width
        data.height = height
        data.angle = 0
        data.desc_1 = 'red'
        data.desc_2 = 'simulation'

        return data

    def show_fence_position(self):
        pass

    def show_path_finder_position(self, posData):
        rospy.loginfo(self.displayed)
        if self.displayed == 1:
            path_position_x = self.posx_sub
            path_position_y = self.posy_sub

            posx = posData.pose.pose.position.x
            posy = posData.pose.pose.position.y
            posz = posData.pose.pose.position.z
            yaw = posData.pose.pose.orientation.z

            self.rect_height += (posz - self.last_position_z) * float(self.pixel_to_meter) / 2.0
            self.rect_width += (posz - self.last_position_z) * float(self.pixel_to_meter) / 2.0

            self.object_width = self.rect_height
            self.object_height = self.rect_width

            self.angle += (yaw - self.last_yaw)

            self.posx_sub = (path_position_x - (posx - self.last_position_x) * self.pixel_to_meter)
            self.posy_sub = (path_position_y - (posy - self.last_position_y) * self.pixel_to_meter)

            self.last_position_x = posx
            self.last_position_y = posy
            self.last_position_z = posz
            self.last_yaw = yaw

            self.pixel_to_meter = (self.rect_height / self.path_width + self.rect_height / self.path_height) / 2

            data = self.set_data(self.posx_sub, self.posy_sub, self.rect_width, self.rect_height, int(self.angle))

            self._publish_image_data.publish(data)

            rospy.loginfo('test')

            self.update()

    def show_buoys_position(self, posData):
        if self.displayed == 1:
            buoy_position_x = self.posx_sub
            buoy_position_y = self.posy_sub

            posx = posData.pose.pose.position.x
            posy = posData.pose.pose.position.y
            posz = posData.pose.pose.position.z
            yaw = posData.pose.pose.orientation.z

            self.ellipse_height += (posx - self.last_position_x) * float(self.pixel_to_meter) / 2.0
            self.ellipse_width += (posx - self.last_position_x) * float(self.pixel_to_meter) / 2.0

            self.object_width = self.ellipse_width
            self.object_height = self.ellipse_height

            self.angle = 0.0

            delta = yaw - self.last_yaw

            if delta == 0:
                offset = 0
            elif delta < 0:
                offset = 10
            elif delta > 0:
                offset = -10

            self.posx_sub = (buoy_position_x - (posy - self.last_position_y) * self.pixel_to_meter) + offset / self.pixel_to_meter
            self.posy_sub = (buoy_position_y - (posz - self.last_position_z) * self.pixel_to_meter)

            self.last_position_x = posx
            self.last_position_y = posy
            self.last_position_z = posz
            self.last_yaw = yaw

            self.pixel_to_meter = (self.ellipse_width / self.buoy_diameter + self.ellipse_height / self.buoy_diameter) / 2

            data = self.set_data(self.posx_sub, self.posy_sub, self.ellipse_width, self.ellipse_height)

            self._publish_image_data.publish(data)

            self.update()

    def _handle_buoy(self):
        self.Buoy = True
        self.Path_fender = False
        self.Fence = False
        self.object_type = Buoy(self)
        self.pixel_to_meter = (self.ellipse_width / self.buoy_diameter + self.ellipse_height / self.buoy_diameter) / 2
        self.object_width = self.ellipse_width
        self.object_height = self.ellipse_height

    def _handle_fence(self):
        self.Fence = True
        self.Buoy = False
        self.Path_fender = False

    def _handle_path_finder(self):
        self.Path_fender = True
        self.Fence = False
        self.Buoy = False
        self.object_type = Path(self)
        self.pixel_to_meter = (self.rect_width / self.path_width + self.rect_height / self.path_height) / 2
        self.object_width = self.rect_width
        self.object_height = self.rect_height

    def _handle_start_test_triggered(self):
        pass

    def _execute_test(self):
        pass

    def shutdown_plugin(self):
        self._odom_subscriber.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
