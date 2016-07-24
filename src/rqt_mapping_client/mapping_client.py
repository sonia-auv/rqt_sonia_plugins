# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse

from qt_gui.plugin import Plugin
from rqt_mapping_client.main_window import MainWindow


class MappingClient(Plugin):
    """
    Subclass of Plugin to provide interactive bag visualization, playing(publishing) and recording
    """

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(MappingClient, self).__init__(context)
        self.setObjectName('MappingClient')

        args = self._parse_args(context.argv())

        self._widget = MainWindow(context)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (
                ' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group(
            'Options for rqt_mapping_client plugin')
        # Add argument this way:
        # group.add_argument('--clock', action='store_true', help='publish the clock time')

    @staticmethod
    def _parse_args(argv):
        parser = argparse.ArgumentParser(prog='rqt_mapping_client',
                                         add_help=False)
        MappingClient.add_arguments(parser)
        return parser.parse_args(argv)

    def shutdown_plugin(self):
        self._widget.shutdown_all()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
