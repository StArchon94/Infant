from threading import Thread
from time import time

import numpy as np
import rclpy
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import Float64


class InfantTouch(Node):
    def __init__(self):
        super().__init__('infant_touch')
        self.comfort_rate, self.max_dt = self.declare_parameters('', [('comfort_rate', 5), ('max_dt', .06)])
        self.pub_comfort = self.create_publisher(Float64, 'change_comfort', 10)
        self.pub_pressure = self.create_publisher(Float64, 'touch_pressure', 1)

    def start_feeling(self):
        s = Serial(port='/dev/ttyACM0')
        t = time()
        while True:
            try:
                string = s.readline().decode().rstrip()
                t_now = time()
                dt = np.min((t_now - t, self.max_dt.value))
                t = t_now
                x = np.clip(0, 1, int(string) / 1024) * np.pi
            except Exception:
                pass
            else:
                v = np.sin(x) if x < np.pi / 2 else 1 + 1 / np.tan(x)
                p = np.clip(0, 1, x * 4 / np.pi - 3) * 100
                self.pub_comfort.publish(Float64(data=self.comfort_rate.value * v * dt))
                self.pub_pressure.publish(Float64(data=p))


def main():
    rclpy.init()
    infant_touch = InfantTouch()
    Thread(target=infant_touch.start_feeling).start()
    rclpy.spin(infant_touch)
