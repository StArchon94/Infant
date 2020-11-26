from threading import Thread

import numpy as np
import pyaudio
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


class InfantEar(Node):
    def __init__(self):
        super().__init__('infant_ear')
        self.reset = False
        pd_read_only = ParameterDescriptor(read_only=True)
        self.chunk, self.alert, self.delta_vol_lo, self.delta_vol_hi, self.discomfort_rate, self.alert_rate, self.alert_decay = self.declare_parameters('', [('chunk_size', 0, pd_read_only), ('alert', .0), ('delta_vol_lo', 1.6), ('delta_vol_hi', 2.5), ('discomfort_rate', 60), ('alert_rate', 30), ('alert_decay', 3)])
        self.pub_alert = self.create_publisher(Float64, 'ear_alert', 1)
        self.pub_comfort = self.create_publisher(Float64, 'change_comfort', 10)
        self.sub_reset = self.create_subscription(Empty, 'reset', self.reset_callback, 1)

    def start_listening(self):
        p = pyaudio.PyAudio()
        for i in range(p.get_device_count()):
            device_info = p.get_device_info_by_index(i)
            if device_info['name'] == 'USB Device 0x46d:0x825: Audio (hw:0,0)':
                fr = int(device_info['defaultSampleRate'])
                nc = device_info['maxInputChannels']
                device_index = device_info['index']
                break
        dt = .3
        chunk = self.chunk.value if self.chunk.value else int(fr * dt)
        stream = p.open(fr, nc, pyaudio.paInt16, input=True, input_device_index=device_index, frames_per_buffer=chunk)
        avg_vol = None
        total = 0
        while True:
            if self.reset:
                self.alert._value = .0
                self.reset = False
            data = np.frombuffer(stream.read(chunk, exception_on_overflow=False), dtype='<i2').astype(float)
            vol = np.log10(np.mean(data**2))
            avg_vol = (avg_vol * total + vol) / (total + 1) if total else vol
            total += 1
            e1 = vol - avg_vol - self.delta_vol_hi.value
            e2 = vol - avg_vol - self.delta_vol_lo.value
            if e1 > 0:
                self.pub_comfort.publish(Float64(data=-self.discomfort_rate.value * e1 * dt))
            elif e2 > 0:
                self.alert._value += self.alert_rate.value * e2 * dt
            self.alert._value -= self.alert_decay.value * dt
            if self.alert.value < 0:
                self.alert._value = .0
            self.set_parameters([self.alert])
            self.pub_alert.publish(Float64(data=self.alert.value))
        # stream.stop_stream()
        # stream.close()
        # p.terminate()

    def reset_callback(self, msg):
        self.reset = True


def main():
    rclpy.init()
    infant_ear = InfantEar()
    Thread(target=infant_ear.start_listening).start()
    rclpy.spin(infant_ear)
