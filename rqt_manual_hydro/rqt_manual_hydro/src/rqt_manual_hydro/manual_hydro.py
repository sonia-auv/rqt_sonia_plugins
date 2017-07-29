from rqt_gui_py.plugin import Plugin

from manual_hydro_widget import ManualHydroWidget


class ManualHydro(Plugin):

    def __init__(self, context):
        super(ManualHydro, self).__init__(context)
        self.setObjectName('Manual_Vision')

        self._widget = ManualHydroWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        self._widget.setPalette(context._handler._main_window.palette())
        self._widget.setAutoFillBackground(True)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
