import abc

from python_qt_binding.QtWidgets import QAction

class Layer:

    def __init__(self, name,parent_widget):
        self._name = name
        self._is_active = True
        self.menu_action = QAction(self._name, parent_widget, triggered=self.toggle_active, checkable=True,
                        checked=True)

    def toggle_active(self):
        self._is_active = self._is_active != True

    def set_active(self,active):
        self._is_active = active

    def draw(self):
        if self._is_active:
            self._draw()

    # Overrided by chlidren class
    def _draw(self):
        None

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(self._name + '_is_active', str(self._is_active))
        #view_matrix_string = repr(self._gl_view.get_view_matrix())
        #instance_settings.set_value('view_matrix', view_matrix_string)

    def restore_settings(self, plugin_settings, instance_settings):
        is_active = instance_settings.value(self._name + '_is_active')
        if is_active is not None :
            self._is_active = is_active == 'True'
            self.menu_action.setChecked(self._is_active)
        #view_matrix_string = instance_settings.value('view_matrix')
        #try:
        #    view_matrix = eval(view_matrix_string)
        #except Exception:
        #    view_matrix = None

        #if view_matrix is not None:
        #    self._gl_view.set_view_matrix(view_matrix)
        #else:
        #    self._set_default_view()
