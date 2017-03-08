from __future__ import division
import struct

from layer.PathLayer import PathLayer
from layer.GridLayer import GridLayer
from layer.SubmarineLayer import SubmarineLayer
from layer.CoorSystemLayer import CoorSystemLayer
from layer.TargetLayer import TargetLayer

from python_qt_binding.QtCore import QTimer, qWarning

from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glVertex3f,glNormal3f, GL_LINES


# main class inherits from the ui window class
class MapDrawer():

    def __init__(self, mainWidget):
        self.mainWidget = mainWidget
        # create GL view
        self._gl_view = mainWidget._gl_view
        self._gl_view.setAcceptDrops(True)
        self.resolution_meters = 50
        self._cross = None

        # backup and replace original paint method
        self._gl_view.paintGL_original = self._gl_view.paintGL
        self._gl_view.paintGL = self._gl_view_paintGL


        self._grid_layer = GridLayer(self.resolution_meters,self._gl_view)
        self._submarine_layer = SubmarineLayer(self.resolution_meters,self._gl_view)
        self._coor_layer = CoorSystemLayer(self._gl_view)
        self._target_layer = TargetLayer(self._gl_view)
        self._path_layer = PathLayer(self.resolution_meters,self._gl_view)

        self._layers = []
        self._layers.append(self._grid_layer)
        self._layers.append(self._submarine_layer)
        self._layers.append(self._coor_layer)
        self._layers.append(self._target_layer)
        self._layers.append(self._path_layer)

        # backup and replace original mouse release method
        self._gl_view.mouseReleaseEvent_original = self._gl_view.mouseReleaseEvent
        self._gl_view.mouseReleaseEvent = self.mainWidget._gl_view_mouseReleaseEvent

        # init and start update timer with 40ms (25fps)
        self._update_timer = QTimer(mainWidget)
        self._update_timer.timeout.connect(self.update_timeout)
        self._update_timer.start(100)

    def get_layers(self):
        return self._layers
    def set_position(self,position):
        self._submarine_layer.set_position(position)
        self._path_layer.position_update(position)

    def set_orientation(self,orientation,yaw):
        self._submarine_layer.set_orientation(orientation,yaw)

    def _set_default_view(self):
        self._gl_view.makeCurrent()
        self._gl_view.reset_view()
        #self._gl_view.rotate((0, 0, 1), 45)
        #self._gl_view.rotate((1, 0, 0), -65)
        self._gl_view.translate((0, 0, -800))

    def update_timeout(self):
        self._gl_view.makeCurrent()
        self._gl_view.updateGL()

    def _gl_view_paintGL(self):
        self._gl_view.paintGL_original()
        self._grid_layer.draw()
        self._coor_layer.draw()
        self._target_layer.draw()
        self._path_layer.draw()
        self._submarine_layer.draw()

    def reset_path(self):
        self._path_layer.reset_path()

    def drawTarget(self, x, y, z):
        self._target_layer.set_target((x * self.resolution_meters, y * self.resolution_meters, z * self.resolution_meters))

    def set_lock_on_sub_activated(self,activate):
        self._submarine_layer.set_lock_on_sub(activate)

    def is_using_2d_view(self,activate):
        self._submarine_layer.is_2d_view(activate)

    def set_rotate_with_sub_activated(self,activate):
        self._submarine_layer.set_rotate_with_sub_activated(activate)

    def save_settings(self, plugin_settings, instance_settings):
        for layer in self._layers:
            layer.save_settings(plugin_settings,instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        for layer in self._layers:
            layer.restore_settings(plugin_settings,instance_settings)

    def shutdown_plugin(self):
        print 'Shutting down'
