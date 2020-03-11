from threading import Thread
from time import time

import rclpy
from rclpy.node import Node
from serial import Serial
from serial.tools.list_ports import comports
from std_msgs.msg import Float64


class InfantPulse(Node):
    def __init__(self):
        super().__init__('infant_pulse')
        self.recovery_rate, self.recovery_decay, self.timeout = self.declare_parameters('', [('recovery_rate', 10), ('recovery_decay', 9), ('timeout', 1.5)])
        self.pub_recovery = self.create_publisher(Float64, 'change_recovery', 10)

    def start_feeling(self):
        s = Serial(port='/dev/ttyACM1')
        t = time()
        is_pulsing = False
        while True:
            try:
                string = s.readline().decode().rstrip()
                t_now = time()
                dt = t_now - t
                t = t_now
                i = int(string)
            except Exception:
                pass
            else:
                if i >= 768:
                    if not is_pulsing:
                        is_pulsing = True
                    t_last = time()
                elif is_pulsing:
                    delta = time() - t_last
                    if delta > self.timeout.value:
                        is_pulsing = False
                self.pub_recovery.publish(Float64(data=(self.recovery_rate.value * is_pulsing - self.recovery_decay.value) * dt))


def main():
    rclpy.init()
    infant_pulse = InfantPulse()
    Thread(target=infant_pulse.start_feeling).start()
    rclpy.spin(infant_pulse)
