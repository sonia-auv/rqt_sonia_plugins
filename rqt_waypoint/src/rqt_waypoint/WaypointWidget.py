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
from sonia_common.srv import ObjectPoseService, SetSimulationAUVService
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

        self.current_mode_id = 0
        self.z_pose = 0

        self.sendWaypointButton.setEnabled(False)
        self.sendWaypointButton.setText("Choose a mode")

        self.frameChoice.setCurrentIndex(1)

        self.prev_auv = ""
        self.prev_scene = ""
        self.prev_run = ""

        # Subscribers
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', Pose, self._position_target_callback)
        self.controller_info_subscriber = rospy.Subscriber("/proc_control/controller_info", MpcInfo, self.set_mpc_info)
        # self.auv_position_subscriber = rospy.Subscriber("/proc_nav/auv_states", Odometry, self.auv_pose_callback)
        # self.auv_position_subscriber = rospy.Subscriber("/telemetry/auv_states", Odometry, self.auv_pose_callback)

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
        self.initial_position_service = rospy.ServiceProxy("/proc_simulation/auv_pose", ObjectPoseService)
        self.set_auv_service = rospy.ServiceProxy("/proc_simulation/select_auv", SetSimulationAUVService)
        self.depth_tare_service = rospy.ServiceProxy("/provider_depth/tare", Empty)
        self.imu_tare_service = rospy.ServiceProxy("/provider_imu/tare", Empty)

        self.current_target_received.connect(self._current_target_received)

        # Simulation menu
        self.actionStart_Simulation.triggered.connect(self.send_initial_position)
        self.actionReset_Position.triggered.connect(self._reset_position)

        # Sensors menu
        self.actionReset_Depth.triggered.connect(self._reset_depth)
        self.actionTare_IMU.triggered.connect(self._tare_imu)
        self.actionStart_DVL.triggered.connect(self.startDVL)
        self.actionStop_DVL.triggered.connect(self.stopDVL)
        self.actionStart_SONAR.triggered.connect(self.startSonar)
        self.actionStop_SONAR.triggered.connect(self.stopSonar)

        # Waypoint tab buttons
        self.resetTrajectory.clicked.connect(self._clear_waypoint)
        self.sendWaypointButton.clicked.connect(self.send_position)

        # Unity tab buttons
        self.updateUnityButton.clicked.connect(self.update_unity)

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

    def startDVL(self):
        self.set_dvl_started_publisher.publish(data=True)

    def stopDVL(self):
        self.set_dvl_started_publisher.publish(data=False)

    def startSonar(self):
        self.set_sonar_started_publisher.publish(data=True)

    def stopSonar(self):
        self.set_sonar_started_publisher.publish(data=False)

    def _reset_position(self):

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        self.simulation_start_publisher.publish(pose)
        # if self.current_mode_id == 0:
        #     self.set_initial_position_publisher.publish(data=True)
        # else:
        #     self.show_error('Control mode must be 0 to reset position')
    
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

    def update_unity(self):
        if self.subChoice.currentText() != self.prev_auv:
            os.environ["AUV"] = self.subChoice.currentText()
            self.set_auv_service.call(object_name=self.subChoice.currentText())
            self.prev_auv = self.subChoice.currentText()

        if self.sceneChoice.currentText() != self.prev_scene:
            # self.show_error(self.sceneChoice.currentText())
            self.show_error("Scene choice not implented yet")
            self.prev_scene = self.sceneChoice.currentText()

        if self.runChoice.currentText() != self.prev_run:
            # self.show_error(self.sceneChoice.currentText())
            self.show_error("Run choice not implented yet")
            self.prev_run = self.runChoice.currentText()

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
        