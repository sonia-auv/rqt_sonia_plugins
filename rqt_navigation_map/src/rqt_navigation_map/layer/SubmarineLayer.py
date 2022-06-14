import struct
import rospkg
import os
import threading

from .Layer import Layer
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, \
    glVertex3f, glNormal3f, GL_LINES, GL_TRIANGLES, GL_QUADS,glLoadMatrixd,glMatrixMode,GL_MODELVIEW,glGetDoublev,GL_MODELVIEW_MATRIX
from tf.transformations import quaternion_matrix


class createpoint:
    def __init__(self, p, c=(1, 0, 0)):
        self.point_size = 0.5
        self.color = c
        self.x = p[0]
        self.y = p[1]
        self.z = p[2]

    def glvertex(self):
        glVertex3f(self.x, self.y, self.z)


# class for a 3d face on a model
class createtriangle:
    points = None
    normal = None

    def __init__(self, p1, p2, p3, n=None):
        # 3 points of the triangle
        self.points = createpoint(p1), createpoint(p2), createpoint(p3)

        # triangles normal
        self.normal = createpoint(self.calculate_normal(self.points[0], self.points[1], self.points[2]))  # (0,1,0)#

    # calculate vector / edge
    def calculate_vector(self, p1, p2):
        return -p1.x + p2.x, -p1.y + p2.y, -p1.z + p2.z

    def calculate_normal(self, p1, p2, p3):
        a = self.calculate_vector(p3, p2)
        b = self.calculate_vector(p3, p1)
        # calculate the cross product returns a vector
        return self.cross_product(a, b)

    def cross_product(self, p1, p2):
        return (p1[1] * p2[2] - p2[1] * p1[2]), (p1[2] * p2[0]) - (p2[2] * p1[0]), (p1[0] * p2[1]) - (p2[0] * p1[1])


class loader:
    model = []
    vehicle_size_x = 0.762 / 2
    vehicle_size_y = 0.699 / 2
    vehicle_size_z = 0.210 / 2
    is_using_2d_view = False
    _lock = threading.Lock()

    def __init__(self,gl_view):
        self.gl_view = gl_view
        self._lock_on_sub = False
        self._lock_rotate = False

    # return the faces of the triangles
    def get_triangles(self):
        if self.model:
            for face in self.model:
                yield face

    # draw the models faces
    def draw(self, resolution_meter, position, orientation,yaw):

        try:
            self._lock.acquire()

            if self.is_using_2d_view:
                z_pos = 5
            else :
                z_pos = (1 - (position[2]/5)) * resolution_meter

            vehicle_position = (
                position[1] * resolution_meter, position[0] * resolution_meter,
                z_pos)

            glTranslatef(vehicle_position[0],vehicle_position[1],vehicle_position[2])  # Translate Box

            matrix = quaternion_matrix(orientation)  # convert quaternion to translation matrix
            glMultMatrixf(matrix)  # Rotate Box

            glBegin(GL_TRIANGLES)
            glColor3f(0.0078, 0.2588, 0.39607)
            for tri in self.get_triangles():
                glNormal3f(tri.normal.x, tri.normal.z, tri.normal.y)
                glVertex3f((tri.points[0].x - self.vehicle_size_y) * resolution_meter,
                           (tri.points[0].z - self.vehicle_size_x) * resolution_meter,
                           (tri.points[0].y - self.vehicle_size_z) * resolution_meter)
                glVertex3f((tri.points[1].x - self.vehicle_size_y) * resolution_meter,
                           (tri.points[1].z - self.vehicle_size_x) * resolution_meter,
                           (tri.points[1].y - self.vehicle_size_z) * resolution_meter)
                glVertex3f((tri.points[2].x - self.vehicle_size_y) * resolution_meter,
                           (tri.points[2].z - self.vehicle_size_x) * resolution_meter,
                           (tri.points[2].y - self.vehicle_size_z) * resolution_meter)
            glEnd()

            if self._lock_on_sub:
                modelview_matrix = self.gl_view.get_view_matrix()
                modelview_matrix[3] = [vehicle_position[0] * -1 , vehicle_position[1] * -1, modelview_matrix[3][2], modelview_matrix[3][3]]
                self.gl_view.load_view_matrix(modelview_matrix)

            if self._lock_rotate:
                self.gl_view.rotate_translate_absolute((0, 0, 1), yaw,
                                                           (vehicle_position[0] * -1, vehicle_position[1] * -1, 0))


        finally:
                self._lock.release()

    def is_2d_view(self,is_2d_view):
        self.is_using_2d_view = is_2d_view

    def set_lock_on_sub(self,activate):
        self._lock_on_sub = activate

    def set_rotate_with_sub_activated(self,activate):
        self._lock_rotate = activate
    # load stl file detects if the file is a text file or binary file
    def load_stl(self, filename):
        # read start of file to determine if its a binay stl file or a ascii stl file
        fp = open(filename, 'rb')
        h = fp.read(80)
        type = h[0:5]
        fp.close()
        self.load_binary_stl(filename)

    # load binary stl file check wikipedia for the binary layout of the file
    # we use the struct library to read in and convert binary data into a format we can use
    def load_binary_stl(self, filename):
        fp = open(filename, 'rb')
        h = fp.read(80)

        l = struct.unpack('I', fp.read(4))[0]
        count = 0
        while True:
            try:
                p = fp.read(12)
                if len(p) == 12:
                    n = struct.unpack('f', p[0:4])[0], struct.unpack('f', p[4:8])[0], struct.unpack('f', p[8:12])[0]

                p = fp.read(12)
                if len(p) == 12:
                    p1 = struct.unpack('f', p[0:4])[0], struct.unpack('f', p[4:8])[0], struct.unpack('f', p[8:12])[0]

                p = fp.read(12)
                if len(p) == 12:
                    p2 = struct.unpack('f', p[0:4])[0], struct.unpack('f', p[4:8])[0], struct.unpack('f', p[8:12])[0]

                p = fp.read(12)
                if len(p) == 12:
                    p3 = struct.unpack('f', p[0:4])[0], struct.unpack('f', p[4:8])[0], struct.unpack('f', p[8:12])[0]

                new_tri = (n, p1, p2, p3)

                if len(new_tri) == 4:
                    tri = createtriangle(p1, p2, p3, n)
                    self.model.append(tri)
                count += 1
                fp.read(2)

                if len(p) == 0:
                    break
            except EOFError:
                break
        fp.close()


class SubmarineLayer(Layer):
    def __init__(self, resolution_meter, parent_widget):
        Layer.__init__(self, 'Submarine Layer', parent_widget)
        self._resolution_meters = resolution_meter
        self.subModel = loader(parent_widget)
        stl_file = os.path.join(rospkg.RosPack().get_path('rqt_navigation_map'), 'resource', 'sub.stl')
        self.subModel.load_binary_stl(stl_file)

    def set_position(self, position):
        self._position = position

    def set_orientation(self, orientation,yaw):
        self._orientation = orientation
        self._yaw = yaw

    def _draw(self):
        self.subModel.draw(self._resolution_meters, self._position, self._orientation,self._yaw)

    def set_lock_on_sub(self,activate):
        self.subModel.set_lock_on_sub(activate)

    def set_rotate_with_sub_activated(self,activate):
        self.subModel.set_rotate_with_sub_activated(activate)

    def is_2d_view(self,is_2d_view):
        self.subModel.is_2d_view(is_2d_view)
