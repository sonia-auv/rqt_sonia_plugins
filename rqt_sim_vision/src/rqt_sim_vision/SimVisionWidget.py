import os
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget
from python_qt_binding.QtGui import QPainter, QColor, QPen, QBrush
from python_qt_binding.QtCore import pyqtSignal
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class SimVisionWidget(QMainWindow):
    odom_result_data = pyqtSignal(Odometry)
    width = 300
    height = 300

    buoy_diameter = 0.23

    pixel_to_meter = (width / buoy_diameter + height / buoy_diameter) / 2

    def __init__(self):
        super(SimVisionWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('SimVisionWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_sim_vision'), 'resource', 'mainwindow.ui')

        loadUi(ui_file, self)

        self.odom_result_data.connect(self.show_buoys_position)

        self.mousePressEvent = self.my_mouse_press_event

        self.displayed = 0

        self.position_x = 0
        self.position_y = 0

        self.last_position_y = 0
        self.last_position_z = 0

        self.painter = QPainter()

        self.posx_sub = -20
        self.posy_sub = -20

        self.scale_value = 0

        self._odom_subscriber = rospy.Subscriber('/proc_navigation/odom', Odometry, self._odom_callback)

        self._publish_image_data = rospy.Publisher('/proc_image_processing/data', VisionTarget, queue_size=10)

        self.setObjectName('MySimVisionWidget')

    def _odom_callback(self, data):
        self.odom_result_data.emit(data)

    def my_mouse_press_event(self, mouseEvent):
        self.displayed = 1
        self.position_x = mouseEvent.x()
        self.position_y = mouseEvent.y()
        self.posx_sub = self.position_x
        self.posy_sub = self.position_y
        self.update()
        print mouseEvent.x() - 10
        print mouseEvent.y() - 10

    def paintEvent(self, event):
        self.painter.begin(self)
        self.painter.setRenderHint(QPainter.Antialiasing)
        self.painter.setPen(QColor(0, 0, 0))

        sub_x = self.posx_sub
        sub_y = self.posy_sub

        self.painter.setBrush(QBrush(QColor(100, 100, 255)))
        self.painter.drawEllipse(sub_x - self.width / 2, sub_y - self.width/2, self.width, self.height)

        self.painter.end()

    def set_data(self, pos_x, pos_y):
        data = VisionTarget()

        data.header = 'simulation'
        data.x = pos_y
        data.y = pos_x
        data.width = self.width
        data.height = self.height
        data.angle = 0.0
        data.desc_1 = 'simulation'
        data.desc_2 = 'simulation'

        return data

    def show_buoys_position(self, posData):
        if self.displayed == 1:
            buoy_position_x = self.posx_sub
            buoy_position_y = self.posy_sub

            posy = posData.pose.pose.position.y
            posz = posData.pose.pose.position.z

            self.posx_sub = buoy_position_x - (posy - self.last_position_y) * self.pixel_to_meter
            self.posy_sub = buoy_position_y - (posz - self.last_position_z) * self.pixel_to_meter

            self.last_position_y = posy
            self.last_position_z = posz
            
            data = self.set_data(self.posx_sub, self.posy_sub)

            self._publish_image_data.publish(data)

            self.update()
            #posy = self.data_y
            #posz = self.data_z
            #mouse_x = self.position_x
            #mouse_y = self.position_y
            #posx_bouy_sub = 300 + (mouse_x - (posy * self.pixel_to_meter + 300))
            #posy_bouy_sub = 250 + (mouse_y - (posz * self.pixel_to_meter + 250))
            #
            #data = self.set_data(posx_bouy_sub, posy_bouy_sub)
            #self.posx_sub = posx_bouy_sub
            #self.posy_sub = posy_bouy_sub
            #
            #self._publish_image_data.publish(data)
            #
            #self.update()
           
    def _handle_start_test_triggered(self):
        pass

    def _execute_test(self):
        pass

    def shutdown_plugin(self):
        self._publish_image_data.unregister()
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
