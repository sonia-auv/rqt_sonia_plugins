import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow

from proc_control.msg import PositionTarget
from proc_control.srv import SetPositionTarget
from proc_navigation.srv import SetDepthOffset, SetWorldXYOffset
from nav_msgs.msg import Odometry


class WaypointWidget(QMainWindow):
    def __init__(self):
        super(WaypointWidget, self).__init__()
        # Give QObjects reasonable names
        try:
            rospy.wait_for_service('/proc_control/set_global_target', timeout=2)
            rospy.wait_for_service('/proc_navigation/set_depth_offset',timeout=2)
            rospy.wait_for_service('/proc_navigation/set_world_x_y_offset', timeout=2)
        except rospy.ROSException:
            False

        self.setObjectName('WaypointWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_waypoint'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyWaypointWidget')

        self._odom_subscriber = rospy.Subscriber('/proc_navigation/odom', Odometry, self._odom_callback)
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', PositionTarget,
                                                           self._position_target_callback)

        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)
        self.set_initial_depth = rospy.ServiceProxy('/proc_navigation/set_depth_offset', SetDepthOffset)
        self.set_initial_position = rospy.ServiceProxy('/proc_navigation/set_world_x_y_offset', SetWorldXYOffset)

        self.xPositionTarget.returnPressed.connect(self.send_position)
        self.yPositionTarget.returnPressed.connect(self.send_position)
        self.zPositionTarget.returnPressed.connect(self.send_position)
        self.rollPositionTarget.returnPressed.connect(self.send_position)
        self.pitchPositionTarget.returnPressed.connect(self.send_position)
        self.yawPositionTarget.returnPressed.connect(self.send_position)

        self.actionReset_Depth.triggered.connect(self._reset_depth)
        self.actionReset_Position.triggered.connect(self._reset_position)

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
        self.xPositionCurrent.setText('%.2f' % data.pose.pose.position.x)
        self.yPositionCurrent.setText('%.2f' % data.pose.pose.position.y)
        self.zPositionCurrent.setText('%.2f' % data.pose.pose.position.z)
        self.rollPositionCurrent.setText('%.2f' % data.pose.pose.orientation.x)
        self.pitchPositionCurrent.setText('%.2f' % data.pose.pose.orientation.y)
        self.yawPositionCurrent.setText('%.2f' % data.pose.pose.orientation.z)

    def _position_target_callback(self,data):
        print 'received target'
        self.xPositionTarget.setText('%.2f' % data.X)
        self.yPositionTarget.setText('%.2f' % data.Y)
        self.zPositionTarget.setText('%.2f' % data.Z)
        self.rollPositionTarget.setText('%.2f' % data.ROLL)
        self.pitchPositionTarget.setText('%.2f' % data.PITCH)
        self.yawPositionTarget.setText('%.2f' % data.YAW)

    def send_position(self):
        print 'Sending position'
        x_target = float(self.xPositionTarget.text())
        y_target = float(self.yPositionTarget.text())
        z_target = float(self.zPositionTarget.text())
        roll_target = float(self.rollPositionTarget.text())
        pitch_target = float(self.pitchPositionTarget.text())
        yaw_target = float(self.yawPositionTarget.text())
        try:
            self.set_global_target(X=x_target, Y=y_target, Z=z_target, ROLL=roll_target, PITCH=pitch_target, YAW=yaw_target)
        except rospy.ServiceException as err:
            rospy.logerr(err)
