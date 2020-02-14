import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QMainWindow, QToolBar

from ToolbarEnableAxisWidget import EnableAxisWidget
from ToolbarBatteryWidget import BatteryWidget
from ToolbarCpuTempWidget import CpuTempWidget
from ToolbarKillmissionWidget import KillMissionWidget
from ToolbarCameraWidget import CameraWidget
from Palette import Palette


class ToolBar(Plugin):

    def __init__(self, context):
        super(ToolBar, self).__init__(context)


        # Give QObjects reasonable namesBatteryWidget
        self.setObjectName('EnableAxis')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._toolbar = QToolBar()
        self._palette = Palette()
        self._enableAxisWidget = EnableAxisWidget()
        self._killMissionWidget = KillMissionWidget()
        self._camera = CameraWidget()
        context._handler._main_window.setPalette(self._palette.palette())
        self._batteryWidget1 = BatteryWidget(1, 1)
        self._batteryWidget2 = BatteryWidget(2, 3)
        self._tempWidget1 = CpuTempWidget('/provider_system/system_temperature', 'Babe')

        # Add widget to the user interface
        self._toolbar.addWidget(self._enableAxisWidget)
        self._toolbar.addWidget(self._camera)
        self._toolbar.addWidget(self._tempWidget1)
        self._toolbar.addWidget(self._batteryWidget1)
        self._toolbar.addWidget(self._batteryWidget2)
        self._toolbar.addWidget(self._killMissionWidget)

        context.add_toolbar(self._toolbar)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._killMissionWidget.shutdown_plugin()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
