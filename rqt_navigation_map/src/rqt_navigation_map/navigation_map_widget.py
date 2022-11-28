from __future__ import division
import os
from turtle import position
import rospkg
import math
import numpy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QPoint, pyqtSignal
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

import rospy
from tf.transformations import quaternion_about_axis, euler_from_quaternion

from .gl_widget import GLWidget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from .MapDrawer import MapDrawer
from sonia_common.msg import AddPose, MultiAddPose, MpcInfo


# main class inherits from the ui window class
class NavigationMapWidget(QWidget):

    odom_result_received = pyqtSignal('PyQt_PyObject')
    current_target_received = pyqtSignal('PyQt_PyObject')


    def __init__(self, plugin):
        super(NavigationMapWidget, self).__init__()
        rp = rospkg.RosPack()

        ui_file = os.path.join(rp.get_path('rqt_navigation_map'), 'resource', 'mainWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        self._topic_name = None
        # self._odom_subscriber = None

        # create GL view
        self._gl_view = GLWidget()
        self._gl_view.unsetCursor()
        self._mapDrawer = MapDrawer(self)

        self._position = (0.0, 0.0, 0.0)
        self._mapDrawer.set_position(self._position)
        self._orientation = quaternion_about_axis(45.0, (0.0, 0.0, 1.0))
        self._mapDrawer.set_orientation(self._orientation,45)
        # add GL view to widget layout
        self.layout().addWidget(self._gl_view)

        self._rotate_with_sub_activated = False
        self._lock_on_sub_activated = False
        self._yaw = 0

        self.controller_info_subscriber = rospy.Subscriber("/proc_control/controller_info", MpcInfo, self.set_mpc_info)
        self._odom_subscriber = rospy.Subscriber('/proc_nav/auv_states', Odometry, self._odom_callback)
        #self._odom_subscriber = rospy.Subscriber('/telemetry/auv_states', Odometry, self._odom_callback)
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', Pose,
                                                           self._position_target_callback)

        self.odom_result_received.connect(self._odom_callback_signal)
        self.current_target_received.connect(self._position_target_callback_signal)

        self.single_add_pose_publisher = rospy.Publisher("/proc_control/add_pose", AddPose, queue_size=10)
        self.multi_add_pose_publisher = rospy.Publisher("/proc_planner/send_multi_addpose", MultiAddPose, queue_size=10)
        
        self._define_menu()

    def _define_menu(self):
        self._menu = QMenu(self._gl_view)

        layer_menu = QMenu('Layers', self._gl_view)
        for layer in self._mapDrawer.get_layers():
            layer_menu.addAction(layer.menu_action)
        self._menu.addMenu(layer_menu)
        self._menu.addSeparator()

        self._view2dAction = QAction(self._gl_view.tr("2D View"), self._gl_view, triggered=self._set_default_view)
        self._menu.addAction(self._view2dAction)

        self._view3dAction = QAction(self._gl_view.tr("3D View"), self._gl_view, triggered=self._set_3d_view)
        self._menu.addAction(self._view3dAction)
        self._menu.addSeparator()

        self._rotateSubAction = QAction(self._gl_view.tr("Rotate with Sub"), self, checkable=True,
                               triggered=self._rotate_with_sub)
        self._menu.addAction(self._rotateSubAction)

        self._lockOnSubAction = QAction(self._gl_view.tr("Lock on Sub"), self, checkable=True,
                                        triggered=self._lock_on_sub)
        self._menu.addAction(self._lockOnSubAction)
        self._menu.addSeparator()

        self._menu.addAction(QAction('Reset Path', self, triggered=self._mapDrawer.reset_path))

        setLocationAction = QAction('Set Location Target', self._gl_view, triggered=self._set_location_action)
        self._menu.addAction(setLocationAction)

    def _position_target_callback(self, target):
        self.current_target_received.emit(target)

    def _position_target_callback_signal(self, target):
        self._mapDrawer.drawTarget(target.position.x, target.position.y, target.position.z)
        self._current_target_z = target.position.z
        self._current_target_yaw = math.degrees(euler_from_quaternion([target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w],'szyx')[0])

    def _odom_callback_signal(self,odom_data):
        self.vehicle_position_x = odom_data.pose.pose.position.x
        self.vehicle_position_y = odom_data.pose.pose.position.y
        self.vehicle_position_z = odom_data.pose.pose.position.z
        self.vehicle_orientation_x = odom_data.pose.pose.orientation.y
        self.vehicle_orientation_y = odom_data.pose.pose.orientation.x
        self.vehicle_orientation_z = odom_data.pose.pose.orientation.z
        self.vehicle_orientation_w = odom_data.pose.pose.orientation.w
        self._position = (self.vehicle_position_x, self.vehicle_position_y, self.vehicle_position_z)
        self._mapDrawer.set_position(self._position)
        self._yaw = math.degrees(euler_from_quaternion([self.vehicle_orientation_x,self.vehicle_orientation_y,self.vehicle_orientation_z,self.vehicle_orientation_w],'szyx')[0])
        self._orientation = (self.vehicle_orientation_x,self.vehicle_orientation_y,self.vehicle_orientation_z,self.vehicle_orientation_w)
        self._mapDrawer.set_orientation(self._orientation, self._yaw)

    def _odom_callback(self, odom_data):
        self.odom_result_received.emit(odom_data)

    def set_mpc_info(self, msg):
        self.current_mode_id = msg.mpc_mode

    def save_settings(self, plugin_settings, instance_settings):
        self._mapDrawer.save_settings(plugin_settings,instance_settings)
        view_matrix_string = repr(self._gl_view.get_view_matrix())
        instance_settings.set_value('view_matrix', view_matrix_string)
        instance_settings.set_value('lock_on_sub_activated', str(self._lock_on_sub_activated))
        instance_settings.set_value('rotate_with_sub_activated', str(self._rotate_with_sub_activated))

    def restore_settings(self, plugin_settings, instance_settings):
        self._mapDrawer.restore_settings(plugin_settings,instance_settings)
        view_matrix_string = instance_settings.value('view_matrix')
        try:
            view_matrix = eval(view_matrix_string)
        except Exception:
            view_matrix = None

        if view_matrix is not None:
            self._gl_view.set_view_matrix(view_matrix)
        else:
            self._set_default_view()

        lock_on_sub = instance_settings.value('lock_on_sub_activated') == 'True'
        if lock_on_sub is None:
            print('Nothing stored for lock_on_sub')
        else:
            self._lock_on_sub(lock_on_sub)
            self._lockOnSubAction.setChecked(lock_on_sub)

        rotate_with_sub = instance_settings.value('rotate_with_sub_activated') == 'True'
        if rotate_with_sub is None:
            print('Nothing stored for lock_on_sub')
        else:
            self._rotate_with_sub(rotate_with_sub)
            self._rotateSubAction.setChecked(rotate_with_sub)

    def _set_default_view(self):
        self._gl_view.makeCurrent()
        self._gl_view.reset_view()
        self._gl_view.translate((0, 0, -800))
        self._mapDrawer.is_using_2d_view(True)

    def _set_3d_view(self):
        self._gl_view.makeCurrent()
        self._gl_view.reset_view()
        self._gl_view.rotate((0, 0, 1), 0)
        self._gl_view.rotate((1, 0, 0), -75)
        self._gl_view.translate((-100, -100, -500))
        self._mapDrawer.is_using_2d_view(False)

    def _rotate_with_sub(self, checked):
        self._lock_on_sub(checked)
        self._lockOnSubAction.setChecked(checked)
        self._rotate_with_sub_activated = checked
        self._mapDrawer.set_rotate_with_sub_activated(checked)

    def _lock_on_sub(self, checked):
        self._lock_on_sub_activated = checked
        self._mapDrawer.set_lock_on_sub_activated(checked)

        self._view2dAction.setEnabled(not checked)
        self._view3dAction.setEnabled(not checked)


    def _gl_view_mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self._event_3dPoint = event
            self._menu.exec_(self._gl_view.mapToGlobal(event.pos()))

    def _set_location_action(self):
        x_cursor = self._event_3dPoint.pos().x()
        y_cursor = self._gl_view.height() - self._event_3dPoint.pos().y()

        x, y, z = self._gl_view.unproject_mouse_on_scene(QPoint(x_cursor, y_cursor))

        position_x = y / self._mapDrawer.resolution_meters
        position_y = x / self._mapDrawer.resolution_meters
        position_z = z / self._mapDrawer.resolution_meters
        self._mapDrawer.drawTarget(position_x, position_y, position_z)
        rospy.loginfo('Set Target selected at (%.2f, %.2f)', position_x, position_y)
        try:
            pose = AddPose()
            pose.position.x = position_x
            pose.position.y = position_y
            pose.position.z = self._current_target_z
            pose.orientation.z = self._current_target_yaw
            pose.frame = 0
            if self.current_mode_id == 11:
                pose.speed = numpy.uint8(abs(self.vehicle_position_x)+abs(position_x) + abs(self.vehicle_position_y)+abs(position_y))
                rospy.loginfo("%s, %s, %s, %s\n%s",position_x,position_y,position_z,pose.speed,pose)
                self.single_add_pose_publisher.publish(pose)
            elif self.current_mode_id == 10:
                multi_pose = MultiAddPose()
                pose.speed = 0
                multi_pose.pose.append(pose)
                multi_pose.interpolation_method = 0
                rospy.loginfo("%s, %s, %s, %s\n%s",position_x,position_y,position_z,pose.speed,pose)
                self.multi_add_pose_publisher.publish(multi_pose)
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def shutdown_plugin(self):
        self._odom_subscriber.unregister()
        self.position_target_subscriber.unregister()
        self.controller_info_subscriber.unregister()
        self.position_target_subscriber.unregister()
        self.single_add_pose_publisher.unregister()
        self.multi_add_pose_publisher.unregister()
