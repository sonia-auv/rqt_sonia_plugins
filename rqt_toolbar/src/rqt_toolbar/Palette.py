import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class Palette(QWidget):

    #This is used in toolbar to set the maiun window palette
    def __init__(self):
        super(Palette, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_toolbar'), 'resource', 'palette.ui')
        loadUi(ui_file, self)

