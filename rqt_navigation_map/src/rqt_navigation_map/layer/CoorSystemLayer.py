from .Layer import Layer
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, \
    glVertex3f,glNormal3f, GL_LINES


class CoorSystemLayer(Layer):
    def __init__(self,parent_widget):
        Layer.__init__(self,'Coordinate Layer',parent_widget)

    def _draw(self):
        glLineWidth(4.0)

        glBegin(GL_LINES)

        glColor3f(0.5, 0.5, 0.5)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(50.0, 0.0, 1.0)

        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 50.0, 1.0)

        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 50.0)

        glEnd()