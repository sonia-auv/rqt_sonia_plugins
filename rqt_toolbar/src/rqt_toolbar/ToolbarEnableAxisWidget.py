import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QColor

from sonia_msgs.srv import EnableControl, ClearWaypoint, SetPositionTarget


class EnableAxisWidget(QWidget):

    def __init__(self):
        super(EnableAxisWidget, self).__init__()
        # Give QObjects reasonable names
        try:
            rospy.wait_for_service('/proc_control/enable_control', timeout=2)
        except rospy.ROSException:
            False
            # do nothing

        self.setObjectName('EnableAxisWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'enable_axis.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self.enable_axis = rospy.ServiceProxy('/proc_control/enable_control', EnableControl)
        self.clear_waypoint_srv = rospy.ServiceProxy('/proc_control/clear_waypoint', ClearWaypoint)
        self.new_toolbar_is_load = True

        # Subscribe to slot
        self.allAxis.clicked[bool].connect(self._handle_allAxis_clicked)
        self.xyAxis.clicked[bool].connect(self._handle_xyAxis_clicked)
        self.depthAxis.clicked[bool].connect(self._handle_depthAxis_clicked)
        self.rollAxis.clicked[bool].connect(self._handle_rollAxis_clicked)
        self.pitchAxis.clicked[bool].connect(self._handle_pitchAxis_clicked)
        self.yawAxis.clicked[bool].connect(self._handle_yawAxis_clicked)

        self.allAxis.click()

        self.new_toolbar_is_load = False

    def _clear_waypoint(self):
        try:
            self.clear_waypoint_srv()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _handle_allAxis_clicked(self, checked):
        self._clear_waypoint()
        if self.xyAxis.isChecked() != checked:
            self.xyAxis.toggle()
        if self.depthAxis.isChecked() != checked:
            self.depthAxis.toggle()
        if self.rollAxis.isChecked() != checked:
            self.rollAxis.toggle()
        if self.pitchAxis.isChecked() != checked:
            self.pitchAxis.toggle()
        if self.yawAxis.isChecked() != checked:
            self.yawAxis.toggle()

        if checked:
            self.xyAxis.setPalette(self.paletteChecked.palette())
            self.depthAxis.setPalette(self.paletteChecked.palette())
            self.rollAxis.setPalette(self.paletteChecked.palette())
            self.pitchAxis.setPalette(self.paletteChecked.palette())
            self.yawAxis.setPalette(self.paletteChecked.palette())
            self.allAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=1, Y=1, Z=1, PITCH=1, ROLL=1, YAW=1)
        else:
            self.xyAxis.setPalette(self.paletteUnchecked.palette())
            self.depthAxis.setPalette(self.paletteUnchecked.palette())
            self.pitchAxis.setPalette(self.paletteUnchecked.palette())
            self.rollAxis.setPalette(self.paletteUnchecked.palette())
            self.yawAxis.setPalette(self.paletteUnchecked.palette())
            self.allAxis.setPalette(self.paletteUnchecked.palette())
            if not self.new_toolbar_is_load:
                self._enable_axis(X=0, Y=0, Z=0, PITCH=0, ROLL=0, YAW=0)

    def _handle_xyAxis_clicked(self, checked):
        if checked:
            self.xyAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=1, Y=1, Z=-1, PITCH=-1, ROLL=-1, YAW=-1)
        else:
            self.xyAxis.setPalette(self.paletteUnchecked.palette())
            self._enable_axis(X=0, Y=0, Z=-1, PITCH=-1, ROLL=-1, YAW=-1)

    def _handle_depthAxis_clicked(self, checked):
        if checked:
            self.depthAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=1, PITCH=-1, ROLL=-1, YAW=-1)
        else:
            self.depthAxis.setPalette(self.paletteUnchecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=0, PITCH=-1, ROLL=-1, YAW=-1)

    def _handle_rollAxis_clicked(self, checked):
        if checked:
            self.rollAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=-1, ROLL=1, YAW=-1)
        else:
            self.rollAxis.setPalette(self.paletteUnchecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=-1, ROLL=0, YAW=-1)

    def _handle_pitchAxis_clicked(self, checked):
        if checked:
            self.pitchAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=1, ROLL=-1, YAW=-1)
        else:
            self.pitchAxis.setPalette(self.paletteUnchecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=0, ROLL=-1, YAW=-1)

    def _handle_yawAxis_clicked(self, checked):
        if checked:
            self.yawAxis.setPalette(self.paletteChecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=-1, ROLL=-1, YAW=1)
        else:
            self.yawAxis.setPalette(self.paletteUnchecked.palette())
            self._enable_axis(X=-1, Y=-1, Z=-1, PITCH=-1, ROLL=-1, YAW=0)

    def _enable_axis(self, X, Y, Z, PITCH, ROLL, YAW):
        try:
            self.enable_axis(X=X, Y=Y, Z=Z, PITCH=PITCH, ROLL=ROLL, YAW=YAW)
        except rospy.ServiceException as err:
            rospy.logerr(err)
