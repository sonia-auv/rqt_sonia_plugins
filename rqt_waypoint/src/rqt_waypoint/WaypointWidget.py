import os
import rospy
import rospkg
import math

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QMessageBox
from python_qt_binding.QtCore import pyqtSignal

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sonia_common.msg import AddPose, MultiAddPose, MpcInfo
from sonia_common.srv import ObjectPoseService, ObjectPoseServiceResponse
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

class WaypointWidget(QMainWindow):

    current_target_received = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(WaypointWidget, self).__init__()
        # Give QObjects reasonable names

        self.setObjectName('WaypointWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_waypoint'), 'resource', 'Mainwindow.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Waypoint')
        self.setObjectName('MyWaypointWidget')

        self.current_mode_id = 0
        self.z_pose = 0

        self.dvl_started = False
        self.actionToggle_DVL.setText("Stop DVL")
        self.sonar_started = False
        self.actionToggle_Sonar.setText("Stop sonar")

        # Subscribers
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', Pose, self._position_target_callback)
        self.controller_info_subscriber = rospy.Subscriber("/proc_control/controller_info", MpcInfo, self.set_mpc_info)
        # self.auv_position_subscriber = rospy.Subscriber("/proc_nav/auv_states", Odometry, self.auv_pose_callback)
        #self.auv_position_subscriber = rospy.Subscriber("/telemetry/auv_states", Odometry, self.auv_pose_callback)

        self.dvl_started_subscriber = rospy.Subscriber("/provider_dvl/enable_disable_ping", Bool, self.dvl_started_callback)
        self.sonar_started_subscriber = rospy.Subscriber("/provider_sonar/enable_disable_ping", Bool, self.sonar_started_callback)

        # Publishers
        self.simulation_start_publisher = rospy.Publisher("/proc_simulation/start_simulation", Pose, queue_size=10, latch=True)
        self.single_add_pose_publisher = rospy.Publisher("/proc_control/add_pose", AddPose, queue_size=10)
        self.multi_add_pose_publisher = rospy.Publisher("/proc_planner/send_multi_addpose", MultiAddPose, queue_size=10)
        self.reset_trajectory_publisher = rospy.Publisher("/proc_control/reset_trajectory", Bool, queue_size=10)
        self.auv7_tare_publisher = rospy.Publisher("/provider_dvl/setDepthOffset", Bool, queue_size=10)
        self.set_dvl_started_publisher = rospy.Publisher("/provider_dvl/enable_disable_ping", Bool, queue_size=10, latch=True)
        self.set_sonar_started_publisher = rospy.Publisher("/provider_sonar/enable_disable_ping", Bool, queue_size=10, latch=True)
        self.set_initial_position_publisher = rospy.Publisher("/proc_nav/reset_pos", Bool, queue_size=10)

        # Services
        self.initial_position_service = rospy.ServiceProxy("obj_pose_srv", ObjectPoseService)
        self.depth_tare_service = rospy.ServiceProxy("/provider_depth/tare", Empty)
        self.imu_tare_service = rospy.ServiceProxy("/provider_imu/tare", Empty)

        self.current_target_received.connect(self._current_target_received)

        self.actionReset_Depth.triggered.connect(self._reset_depth)
        self.actionTare_IMU.triggered.connect(self._tare_imu)
        self.actionToggle_DVL.triggered.connect(self.toggleDVL)
        self.actionToggle_Sonar.triggered.connect(self.toggleSonar)
        self.actionReset_Position.triggered.connect(self._reset_position)
        self.resetTrajectory.clicked.connect(self._clear_waypoint)
        self.actionStart_Simulation.triggered.connect(self.send_initial_position)
        self.sendWaypointButton.clicked.connect(self.send_position)

        self.sendWaypointButton.setEnabled(False)
        self.sendWaypointButton.setText("Choose a mode")

    def _reset_depth(self):
        # Getting AUV name environnment variable.
        auv_name = os.getenv('AUV')
        if auv_name == "AUV7":
            self.auv7_tare_publisher.publish(data=True)
        elif auv_name == "AUV8":
            try:
                self.depth_tare_service.call()
            except rospy.ServiceException as e:
                print(e)
                rospy.logerr('Provider depth is not started.')
        else:
            rospy.logerr('AUV environment variable not properly set.')

    def _tare_imu(self):
        try:
            self.imu_tare_service.call()
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Provider IMU is not started.')

    def toggleDVL(self):
        self.dvl_started = not self.dvl_started
        self.set_dvl_started_publisher.publish(data=self.dvl_started)

    def dvl_started_callback(self, msg):
        self.dvl_started = msg.data
        self.actionToggle_DVL.setText("Stop DVL" if self.dvl_started else "Start DVL")

    def toggleSonar(self):
        self.sonar_started = not self.sonar_started
        self.set_sonar_started_publisher.publish(data=self.sonar_started)

    def sonar_started_callback(self, msg):
        self.sonar_started = msg.data
        self.actionToggle_Sonar.setText("Stop sonar" if self.sonar_started else "Start sonar")

    def _reset_position(self):
        if self.current_mode_id == 0:
            self.set_initial_position_publisher.publish(data=True)
        else:
            self.show_error('Control mode must be 0 to reset position')
    
    def set_mpc_info(self, msg):
        self.current_mode_id = msg.mpc_mode
        if self.current_mode_id != 0:
            self.sendWaypointButton.setText("Send Waypoint")
            self.sendWaypointButton.setEnabled(True)
        else:
            self.sendWaypointButton.setText("Choose a mode")
            self.sendWaypointButton.setEnabled(False)

    # def auv_pose_callback(self, msg):
    #     self.z_pose = float(msg.pose.pose.position.z)
    #     self.z_pose

    def _clear_waypoint(self):
        self.reset_trajectory_publisher.publish(data=True)

    def send_initial_position(self):
        try:
            auv_name = os.getenv('AUV')
            if auv_name:
                resp = self.initial_position_service.call(object_name=auv_name)
                pose = Pose()
                pose.position.x = resp.object_pose.position.x
                pose.position.y = resp.object_pose.position.y
                pose.position.z = resp.object_pose.position.z

                pose.orientation.x = resp.object_pose.orientation.x
                pose.orientation.y = resp.object_pose.orientation.y
                pose.orientation.z = resp.object_pose.orientation.z
                pose.orientation.w = resp.object_pose.orientation.w

                self.simulation_start_publisher.publish(pose)
            else:
                rospy.logerr('AUV environment variable not properly set.')

        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Simulation is not started')
            self.show_error('Simulation is not started')

    def _position_target_callback(self,data):
        self.current_target_received.emit(data)

    def _current_target_received(self, data):
        try:
            self.xPositionCurrent.setText('%.2f' % data.position.x)
            self.yPositionCurrent.setText('%.2f' % data.position.y)
            self.zPositionCurrent.setText('%.2f' % data.position.z)
            self.rollPositionCurrent.setText('%.2f' % math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[2]))
            self.pitchPositionCurrent.setText('%.2f' % math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[1]))
            self.yawPositionCurrent.setText('%.2f' % math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[0]))
        except ValueError:
            pass

    def reset_commands(self):
        self.xPositionTarget.setText('0.0')
        self.yPositionTarget.setText('0.0')
        self.zPositionTarget.setText('0.0')
        self.rollPositionTarget.setText('0.0')
        self.pitchPositionTarget.setText('0.0')
        self.yawPositionTarget.setText('0.0')
        
        self.speed.setText('0')
        self.fine.setText('0.0')

    def send_position(self):
        try:
            print("Sending waypoint.")
            z_axis_problem = False
            x_val = float(self.xPositionTarget.text())
            y_val = float(self.yPositionTarget.text())
            z_val = min(float(self.zPositionTarget.text()), 3)
            roll_val = float(self.rollPositionTarget.text())
            pitch_Val = float(self.pitchPositionTarget.text())
            yaw_val = float(self.yawPositionTarget.text())
            frame_val = self.frameChoice.currentIndex()
            speed_val = int(self.speed.text())
            fine_val = float(self.fine.text())
            method_val = self.methodChoice.currentIndex()
            path_val = self.pathLength.isChecked()
            # Verify z-axis
            if frame_val == 0 or frame_val == 2:
                if z_val > 4: z_axis_problem = True
            # else:
            #     if z_val + self.z_pose > 4: z_axis_problem = True  
            if z_axis_problem:
                self.show_error("Depth too low.")
            else:
                if self.current_mode_id == 11:
                    if speed_val <= 0:
                        self.show_error("Speed incorrect.")
                    else:
                        # Send a single waypoint.
                        pose = AddPose()
                        pose.position.x = x_val
                        pose.position.y = y_val
                        pose.position.z = z_val
                        pose.orientation.x = roll_val
                        pose.orientation.y = pitch_Val
                        pose.orientation.z = yaw_val
                        pose.frame = frame_val
                        pose.speed = speed_val
                        pose.fine = fine_val
                        pose.rotation = path_val

                        self.single_add_pose_publisher.publish(pose)
                        self.reset_commands()

                elif self.current_mode_id == 10:
                    if speed_val < 0 or speed_val > 2:
                        self.show_error("Speed profile incorrect.")
                    else:
                        # Send a multi-waypoint.
                        pose = AddPose()
                        pose.position.x = x_val
                        pose.position.y = y_val
                        pose.position.z = z_val
                        pose.orientation.x = roll_val
                        pose.orientation.y = pitch_Val
                        pose.orientation.z = yaw_val
                        pose.frame = frame_val
                        pose.speed = speed_val
                        pose.fine = fine_val
                        pose.rotation = path_val

                        multi_pose = MultiAddPose()
                        multi_pose.pose.append(pose)
                        multi_pose.interpolation_method = method_val

                        self.multi_add_pose_publisher.publish(multi_pose)
                        self.reset_commands()
        except ValueError:
            pass

    def show_error(self, message):
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Warning)
        msgBox.setText(message)
        msgBox.setWindowTitle("Error")
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec()

    def shutdown_plugin(self):
        self.controller_info_subscriber.unregister()
        self.position_target_subscriber.unregister()
        self.dvl_started_subscriber.unregister()
        self.sonar_started_subscriber.unregister()