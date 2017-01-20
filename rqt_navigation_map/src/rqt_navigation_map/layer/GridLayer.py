import threading
from Layer import Layer
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, \
    glVertex3f,glNormal3f, GL_LINES


class GridLayer(Layer):
    def __init__(self,resolution_meter,parent_widget):
        Layer.__init__(self,'Grid Layer',parent_widget)
        self._resolution_meters = resolution_meter
        self._lock = threading.Lock()

    def _draw(self):
        try:

            self._lock.acquire()
            gridded_area_size = 1500

            glLineWidth(1.0)

            num_of_lines = int(gridded_area_size / self._resolution_meters)
            glBegin(GL_LINES)

            glColor3f(0.188, 0.188, 0.188)

            glVertex3f(gridded_area_size, 0, 0)
            glVertex3f(-gridded_area_size, 0, 0)
            glVertex3f(0, gridded_area_size, 0)
            glVertex3f(0, -gridded_area_size, 0)


            for i in range(num_of_lines):
                glVertex3f(self._resolution_meters * i, -gridded_area_size, 0)
                glVertex3f(self._resolution_meters * i, gridded_area_size, 0)
                glVertex3f(gridded_area_size, self._resolution_meters * i, 0)
                glVertex3f(-gridded_area_size, self._resolution_meters * i, 0)

                glVertex3f(self._resolution_meters * (-i), -gridded_area_size, 0)
                glVertex3f(self._resolution_meters * (-i), gridded_area_size, 0)
                glVertex3f(gridded_area_size, self._resolution_meters * (-i), 0)
                glVertex3f(-gridded_area_size, self._resolution_meters * (-i), 0)

            glEnd()

            glLineWidth(3.0)
            glBegin(GL_LINES)

            glVertex3f(0, -gridded_area_size, 0)
            glVertex3f(0, gridded_area_size, 0)

            glVertex3f(gridded_area_size, 0, 0)
            glVertex3f(-gridded_area_size, 0, 0)

            glEnd()
        finally:
            self._lock.release()