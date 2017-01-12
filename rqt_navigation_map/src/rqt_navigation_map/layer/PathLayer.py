from Layer import Layer
from collections import deque
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, \
    glVertex3f,glNormal3f, GL_LINES


class PathLayer(Layer):
    def __init__(self,resolution_meter,parent_widget):
        Layer.__init__(self,'Path Layer', parent_widget)
        self._resolution_meter = resolution_meter
        self._path_queue = deque( maxlen=60)
        self._last_pos = (0,0,0)

    def reset_path(self):
        self._path_queue.clear()

    def position_update(self,position):
        if position == self._last_pos :
            return

        self._path_queue.append(position)
        self._last_pos = position

    def _draw(self):
        glLineWidth(4.0)

        glBegin(GL_LINES)

        glColor3f(119, 25, 25)
        for path in list(self._path_queue):
            glVertex3f(path[0] * self._resolution_meter -4,path[1] * self._resolution_meter,5)
            glVertex3f(path[0]* self._resolution_meter+4,path[1] * self._resolution_meter,5)

        glEnd()