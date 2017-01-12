import struct
import rospkg
import os

from Layer import Layer
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, \
    glVertex3f, glNormal3f, GL_LINES, GL_TRIANGLES, GL_QUADS
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

    # return the faces of the triangles
    def get_triangles(self):
        if self.model:
            for face in self.model:
                yield face

    # draw the models faces
    def draw(self, resolution_meter, position, orientation):

        vehicle_position = (
            position[0] * resolution_meter, position[1] * resolution_meter,
            position[2] * resolution_meter)
        glTranslatef(*vehicle_position)  # Translate Box

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

    # load stl file detects if the file is a text file or binary file
    def load_stl(self, filename):
        # read start of file to determine if its a binay stl file or a ascii stl file
        print 'open file'
        fp = open(filename, 'rb')
        h = fp.read(80)
        type = h[0:5]
        fp.close()
        print "reading binary stl file " + str(filename, )
        self.load_binary_stl(filename)
        print 'binary stl readed'

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
        self.subModel = loader()
        stl_file = os.path.join(rospkg.RosPack().get_path('rqt_navigation_map'), 'resource', 'sub.stl')
        self.subModel.load_binary_stl(stl_file)

    def set_position(self, position):
        self._position = position

    def set_orientation(self, orientation):
        self._orientation = orientation

    def _draw(self):
        self.subModel.draw(self._resolution_meters, self._position, self._orientation)

    def _draw1(self):
        vehicle_size_x = 0.54 * self._resolution_meters / 2.0
        vehicle_size_y = 1.45 * self._resolution_meters / 2.0
        vehicle_size_z = 0.45 * self._resolution_meters / 2.0
        position = (
            self._position[0] * self._resolution_meters, self._position[1] * self._resolution_meters,
            self._position[2] * self._resolution_meters)
        glTranslatef(*position)  # Translate Box

        matrix = quaternion_matrix(self._orientation)  # convert quaternion to translation matrix
        glMultMatrixf(matrix)  # Rotate Box

        glBegin(GL_QUADS)  # Start Drawing The Box

        glColor3f(0.0, 0.0, 1.0)
        # self.subModel.draw()
        glVertex3f(vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Right Of The Quad (Top)
        glVertex3f(-vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Left Of The Quad (Top)
        glVertex3f(-vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Bottom Left Of The Quad (Top)
        glVertex3f(vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Bottom Right Of The Quad (Top)

        glVertex3f(vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Top Right Of The Quad (Bottom)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Top Left Of The Quad (Bottom)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Left Of The Quad (Bottom)
        glVertex3f(vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Right Of The Quad (Bottom)

        glVertex3f(vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Top Right Of The Quad (Front)
        glVertex3f(-vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Top Left Of The Quad (Front)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Bottom Left Of The Quad (Front)
        glVertex3f(vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Bottom Right Of The Quad (Front)

        glVertex3f(vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Left Of The Quad (Back)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Right Of The Quad (Back)
        glVertex3f(-vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Right Of The Quad (Back)
        glVertex3f(vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Left Of The Quad (Back)

        glVertex3f(-vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Top Right Of The Quad (Left)
        glVertex3f(-vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Left Of The Quad (Left)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Left Of The Quad (Left)
        glVertex3f(-vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Bottom Right Of The Quad (Left)

        glVertex3f(vehicle_size_x, vehicle_size_y, -vehicle_size_z)  # Top Right Of The Quad (Right)
        glVertex3f(vehicle_size_x, vehicle_size_y, vehicle_size_z)  # Top Left Of The Quad (Right)
        glVertex3f(vehicle_size_x, -vehicle_size_y, vehicle_size_z)  # Bottom Left Of The Quad (Right)
        glVertex3f(vehicle_size_x, -vehicle_size_y, -vehicle_size_z)  # Bottom Right Of The Quad (Right)

        glEnd()  # Done Drawing The Quad
        glLineWidth(6.0)

        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0, vehicle_size_y - 2, 0)
        glVertex3f(0, vehicle_size_y + 12, 0)
        glEnd()
