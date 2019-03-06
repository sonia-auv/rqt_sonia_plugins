import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtCore import pyqtSignal

from proc_control.srv import ClearWaypoint, SetPositionTarget, SetControlMode
from proc_navigation.srv import SetDepthOffset, SetWorldXYOffset
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist


class WaypointWidget(QMainWindow):

    odom_result_received = pyqtSignal('PyQt_PyObject')
    current_target_received = pyqtSignal('PyQt_PyObject')
    current_target_velocity_received = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(WaypointWidget, self).__init__()
        # Give QObjects reasonable names
        try:
            rospy.wait_for_service('/proc_control/set_global_target', timeout=2)
            rospy.wait_for_service('/proc_control/clear_waypoint', timeout=2)
            rospy.wait_for_service('/proc_navigation/set_depth_offset', timeout=2)
            rospy.wait_for_service('/proc_navigation/set_world_x_y_offset', timeout=2)
        except rospy.ROSException:
            False

        self.setObjectName('WaypointWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_waypoint'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyWaypointWidget')

        self._odom_subscriber = rospy.Subscriber('/proc_navigation/odom', Odometry, self._odom_callback)
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', Pose,
                                                           self._position_target_callback)

        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target_velocity', Twist,
                                                           self._velocity_target_callback)

        self.odom_result_received.connect(self._odom_result_received)
        self.current_target_received.connect(self._current_target_received)
        self.current_target_velocity_received.connect(self._current_target_velocity_received)

        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)
        self.clear_waypoint_srv = rospy.ServiceProxy('/proc_control/clear_waypoint', ClearWaypoint)
        self.set_initial_depth = rospy.ServiceProxy('/proc_navigation/set_depth_offset', SetDepthOffset)
        self.set_initial_position = rospy.ServiceProxy('/proc_navigation/set_world_x_y_offset', SetWorldXYOffset)

        self.xPositionTarget.returnPressed.connect(self.send_position)
        self.yPositionTarget.returnPressed.connect(self.send_position)
        self.zPositionTarget.returnPressed.connect(self.send_position)
        self.rollPositionTarget.returnPressed.connect(self.send_position)
        self.pitchPositionTarget.returnPressed.connect(self.send_position)
        self.yawPositionTarget.returnPressed.connect(self.send_position)

        self.xVelocityTarget.returnPressed.connect(self.send_velocity)
        self.yVelocityTarget.returnPressed.connect(self.send_velocity)
        self.zVelocityTarget.returnPressed.connect(self.send_velocity)
        self.rollVelocityTarget.returnPressed.connect(self.send_velocity)
        self.pitchVelocityTarget.returnPressed.connect(self.send_velocity)
        self.yawVelocityTarget.returnPressed.connect(self.send_velocity)

        self.actionReset_Depth.triggered.connect(self._reset_depth)
        self.actionReset_Position.triggered.connect(self._reset_position)

        self.clearWaypoint.clicked.connect(self._clear_waypoint)
        self.initialPosition.clicked.connect(self._clear_waypoint_and_reset_position)

    def _reset_depth(self):
        try:
            self.set_initial_depth()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _reset_position(self):
        try:
            self.set_initial_position()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _odom_callback(self, data):
        self.odom_result_received.emit(data)

    def _clear_waypoint(self):
        try:
            self.clear_waypoint_srv()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _clear_waypoint_and_reset_position(self):
        try:
            self.set_initial_position()
            rospy.sleep(0.05)
            self.clear_waypoint_srv()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _odom_result_received(self, odom):
        self.xPositionCurrent.setText('%.2f' % odom.pose.pose.position.x)
        self.yPositionCurrent.setText('%.2f' % odom.pose.pose.position.y)
        self.zPositionCurrent.setText('%.2f' % odom.pose.pose.position.z)
        self.rollPositionCurrent.setText('%.2f' % odom.pose.pose.orientation.x)
        self.pitchPositionCurrent.setText('%.2f' % odom.pose.pose.orientation.y)
        self.yawPositionCurrent.setText('%.2f' % odom.pose.pose.orientation.z)
        self.xVelocityCurrent.setText('%.2f' % odom.twist.twist.linear.x)
        self.yVelocityCurrent.setText('%.2f' % odom.twist.twist.linear.y)
        self.zVelocityCurrent.setText('%.2f' % odom.twist.twist.linear.z)
        self.rollVelocityCurrent.setText('%.2f' % odom.twist.twist.angular.x)
        self.pitchVelocityCurrent.setText('%.2f' % odom.twist.twist.angular.y)
        self.yawVelocityCurrent.setText('%.2f' % odom.twist.twist.angular.z)

    def _position_target_callback(self,data):
        self.current_target_received.emit(data)

    def _velocity_target_callback(self, data):
        self.current_target_velocity_received.emit(data)

    def _current_target_received(self,data):
        try:
            self.xPositionTarget.setText('%.2f' % data.position.x)
            self.yPositionTarget.setText('%.2f' % data.position.y)
            self.zPositionTarget.setText('%.2f' % data.position.z)
            self.rollPositionTarget.setText('%.2f' % data.orientation.x)
            self.pitchPositionTarget.setText('%.2f' % data.orientation.y)
            self.yawPositionTarget.setText('%.2f' % data.orientation.z)
        except ValueError:
            pass

    def _current_target_velocity_received(self,data):
        try:
            self.xVelocityTarget.setText('%.2f' % data.linear.x)
            self.yVelocityTarget.setText('%.2f' % data.linear.y)
            self.zVelocityTarget.setText('%.2f' % data.linear.z)
            self.rollVelocityTarget.setText('%.2f' % data.angular.x)
            self.pitchVelocityTarget.setText('%.2f' % data.angular.y)
            self.yawVelocityTarget.setText('%.2f' % data.angular.z)
        except ValueError:
            pass

    def send_position(self):
        try:
            x_target = float(self.xPositionTarget.text())
            y_target = float(self.yPositionTarget.text())
            z_target = min(float(self.zPositionTarget.text()), 3)
            roll_target = float(self.rollPositionTarget.text())
            pitch_target = float(self.pitchPositionTarget.text())
            yaw_target = float(self.yawPositionTarget.text())
            try:
                self.set_global_target(X=x_target, Y=y_target, Z=z_target, ROLL=roll_target, PITCH=pitch_target,
                                       YAW=yaw_target)
            except rospy.ServiceException as err:
                rospy.logerr(err)
        except ValueError:
            pass

    def send_velocity(self):
        try:
            x_target = float(self.xVelocityTarget.text())
            y_target = float(self.yVelocityTarget.text())
            z_target = min(float(self.zVelocityTarget.text()), 3)
            roll_target = float(self.rollVelocityTarget.text())
            pitch_target = float(self.pitchVelocityTarget.text())
            yaw_target = float(self.yawVelocityTarget.text())
            try:
                self.set_global_target(X=x_target, Y=y_target, Z=z_target, ROLL=roll_target, PITCH=pitch_target,
                                       YAW=yaw_target)
            except rospy.ServiceException as err:
                rospy.logerr(err)
        except ValueError:
            pass



    def shutdown_plugin(self):
        self._odom_subscriber.unregister()
        self.position_target_subscriber.unregister()