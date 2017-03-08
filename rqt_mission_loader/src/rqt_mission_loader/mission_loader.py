from rqt_gui_py.plugin import Plugin

from mission_loader_widget import MissionPlannerWidget


class MissionLoader(Plugin):

    def __init__(self, context):
        super(MissionLoader, self).__init__(context)
        self.setObjectName('MissionLoader')

        self._widget = MissionPlannerWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        self._widget.setPalette(context._handler._main_window.palette())
        self._widget.setAutoFillBackground(True)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()