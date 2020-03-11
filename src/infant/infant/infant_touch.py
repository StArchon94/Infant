from threading import Thread
from time import time

import numpy as np
import rclpy
from rclpy.node import Node
from serial import Serial
from serial.tools.list_ports import comports
from std_msgs.msg import Float64


class InfantTouch(Node):
    def __init__(self):
        super().__init__('infant_touch')
        self.comfort_rate, self.max_dt = self.declare_parameters('', [('comfort_rate', 5), ('max_dt', .06)])
        self.pub_comfort = self.create_publisher(Float64, 'change_comfort', 10)

    def start_feeling(self):
        s = Serial(port='/dev/ttyACM0')
        t = time()
        c = np.pi / 1024
        while True:
            try:
                string = s.readline().decode().rstrip()
                t_now = time()
                dt = np.min((t_now - t, self.max_dt.value))
                t = t_now
                x = c * int(string)
            except Exception:
                pass
            else:
                v = np.sin(x) if x < np.pi / 2 else 1 + 1 / np.tan(x)
                self.pub_comfort.publish(Float64(data=self.comfort_rate.value * v * dt))


def main():
    rclpy.init()
    infant_touch = InfantTouch()
    Thread(target=infant_touch.start_feeling).start()
    rclpy.spin(infant_touch)
