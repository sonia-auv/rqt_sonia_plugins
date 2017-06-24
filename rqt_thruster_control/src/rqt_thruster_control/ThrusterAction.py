import time

from sonia_msgs.msg import SendCanMsg


class ThrusterAction():
    def __init__(self, mainwindow, thruster_unique_id, thrusterName):
        self.request_device_id = SendCanMsg.DEVICE_ID_actuators
        self.thruster_id = thruster_unique_id
        self.thrusterName = thrusterName
        self.mainwindow = mainwindow
        self.request_method_number = SendCanMsg.METHOD_MOTOR_set_speed

        eval('mainwindow.' + thrusterName + '_effort0.clicked[bool]').connect(self._handle_thruster_effort0_clicked)
        eval('mainwindow.' + thrusterName + '_effort50.clicked[bool]').connect(self._handle_thruster_effort50_clicked)
        eval('mainwindow.' + thrusterName + '_effort100.clicked[bool]').connect(self._handle_thruster_effort100_clicked)
        eval('mainwindow.' + thrusterName + '_effortm50.clicked[bool]').connect(self._handle_thruster_effortm50_clicked)
        eval('mainwindow.' + thrusterName + '_effortm100.clicked[bool]').connect(self._handle_thruster_effortm100_clicked)


        eval('mainwindow.' + thrusterName + 'Slider.valueChanged').connect(self._handle_thruster_slider_valueChanged)


        # self.publisher.publish(self.request_device_id, self.request_unique_id[i], self.request_method_number,self.values[i])

    def _handle_thruster_effort0_clicked(self, checked):
        self._update_speed_slider(0)

    def _handle_thruster_effort50_clicked(self, checked):
        self._update_speed_slider(50)

    def _handle_thruster_effort100_clicked(self, checked):
        self._update_speed_slider(100)

    def _handle_thruster_effortm50_clicked(self, checked):
        self._update_speed_slider(-50)

    def _handle_thruster_effortm100_clicked(self, checked):
        self._update_speed_slider(-100)

    def _handle_thruster_slider_valueChanged(self):
        value = eval('self.mainwindow.'+self.thrusterName+'Slider.value()')
        self.mainwindow.publisher.publish(ID=self.thruster_id, effort=value)
        self._update_speed_text(value)

    def run_test(self):
        self._handle_thruster_effort0_clicked(False)
        time.sleep(0.5)
        self._update_speed_slider(10)
        time.sleep(1.5)
        self._handle_thruster_effort0_clicked(False)



    def _update_speed_text(self, value):
        eval('self.mainwindow.' + self.thrusterName + 'Speed').setText(str(value))

    def _update_speed_slider(self, value):
        eval('self.mainwindow.' + self.thrusterName + 'Slider').setValue(value)
