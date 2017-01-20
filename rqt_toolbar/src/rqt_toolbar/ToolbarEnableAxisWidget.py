import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from proc_control.srv import EnableControl


class EnableAxisWidget(QWidget):

    def __init__(self):
        super(EnableAxisWidget, self).__init__()
        # Give QObjects reasonable names
        try:
            rospy.wait_for_service('/proc_control/enable_control',timeout=2)
        except rospy.ROSException:
            False
            # do nothing


        self.setObjectName('EnableAxisWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'enable_axis.ui')
        loadUi(ui_file, self)

        self.setObjectName('MyEnableAxisWidget')
        self.enable_axis = rospy.ServiceProxy('/proc_control/enable_control', EnableControl)

        # Subscribe to slot
        self.allAxis.clicked[bool].connect(self._handle_allAxis_clicked)
        self.xyAxis.clicked[bool].connect(self._handle_xyAxis_clicked)
        self.depthAxis.clicked[bool].connect(self._handle_depthAxis_clicked)
        self.yawAxis.clicked[bool].connect(self._handle_yawAxis_clicked)

    def _handle_allAxis_clicked(self, checked):
        if self.xyAxis.isChecked() != checked:
            self.xyAxis.toggle()
        if self.depthAxis.isChecked() != checked:
            self.depthAxis.toggle()
        if self.yawAxis.isChecked() != checked:
            self.yawAxis.toggle()

        if checked:
            self.enable_axis(X=1, Y=1,Z=1,PITCH=1,ROLL=1, YAW=1)
        else:
            self.enable_axis(X=0, Y=0,Z=0,PITCH=0,ROLL=0, YAW=0)

    def _handle_xyAxis_clicked(self, checked):
        if checked:
            self.enable_axis(X=1, Y=1,Z=-1,PITCH=-1,ROLL=-1, YAW=-1)
        else :

            self.enable_axis(X=0, Y=0,Z=-1,PITCH=-1,ROLL=-1, YAW=-1)

    def _handle_depthAxis_clicked(self, checked):
        if checked:
            self.enable_axis(X=-1, Y=-1,Z=1,PITCH=-1,ROLL=-1, YAW=-1)
        else :
            self.enable_axis(X=-1, Y=-1,Z=0,PITCH=-1,ROLL=-1, YAW=-1)

    def _handle_yawAxis_clicked(self, checked):
        if checked:
            self.enable_axis(X=-1, Y=-1,Z=-1,PITCH=-1,ROLL=-1, YAW=1)
        else :
            self.enable_axis(X=-1, Y=-1,Z=-1,PITCH=-1,ROLL=-1, YAW=0)