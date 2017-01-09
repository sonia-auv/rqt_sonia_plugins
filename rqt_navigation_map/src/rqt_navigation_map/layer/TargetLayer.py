from Layer import Layer
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glVertex3f, GL_LINES


class TargetLayer(Layer):
    def __init__(self,parent_widget):
        Layer.__init__(self, 'Target Layer', parent_widget)
        self._cross = None

    def set_target(self, target):
        self._cross = target

    def _draw(self):
        if self._cross is None:
            return

        glLineWidth(3.0)
        glBegin(GL_LINES)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(self._cross[0] - 20, self._cross[1] - 20, 10)
        glVertex3f(self._cross[0] + 20, self._cross[1] + 20, 10)
        glVertex3f(self._cross[0] - 20, self._cross[1] + 20, 10)
        glVertex3f(self._cross[0] + 20, self._cross[1] - 20, 10)

        glEnd()
