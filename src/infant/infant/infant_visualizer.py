import os
from threading import Thread
from time import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from infant_interfaces.msg import InfantState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64

from .utils import Squeezer, pixelate


class InfantVisualizer(Node):
    def __init__(self, h, w, l=None):
        super().__init__('infant_visualizer')
        self.reset = False
        pd_read_only = ParameterDescriptor(read_only=True)
        self.half_life, self.delta, self.period = self.declare_parameters('', [('half_life', 0.4, pd_read_only), ('delta', .8, pd_read_only), ('period', 3)])
        self.figs = {}
        resource_dir = os.path.join(get_package_share_directory('infant'), 'resource/')
        for stance in ['low', 'normal', 'high']:
            self.figs[stance] = {}
            self.figs[stance]['plain'] = cv2.imread(os.path.join(resource_dir, f'{stance}_plain.jpg'))
            for style in ['rot', 'sat', 'unsat']:
                if style == 'rot' and stance != 'low':
                    continue
                self.figs[stance][style] = [cv2.imread(os.path.join(resource_dir, f'{stance}_{style}/{i}.jpg')) for i in range(1, 101)]
        self.alpha = np.log(2) / self.half_life.value
        self.h, self.w = h, w
        self.l = min(h, w) if l is None else l

        self.sub_state = self.create_subscription(InfantState, 'state', self.state_callback, 1)
        self.sub_focus = self.create_subscription(Float64, 'eye_focus', self.focus_callback, 1)
        self.sub_pressure = self.create_subscription(Float64, 'touch_pressure', self.pressure_callback, 1)
        self.sub_reset = self.create_subscription(Empty, 'reset', self.reset_callback, 1)
        self.alert = None
        self.comfort = None
        self.stage = None
        self.num = None
        self.recovery = None
        self.tgt_focus = None
        self.tgt_pressure = None

    def state_callback(self, msg):
        self.alert = msg.alert
        self.comfort = msg.comfort
        self.stage = msg.stage
        self.num = msg.num
        self.recovery = msg.recovery

    def focus_callback(self, msg):
        self.tgt_focus = msg.data

    def pressure_callback(self, msg):
        self.tgt_pressure = msg.data

    def reset_callback(self, msg):
        self.reset = True

    def start_plotting(self):
        def write_text(img, text):
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 12
            thickness = 24
            textsize = cv2.getTextSize(text, font, font_scale, thickness)[0]
            textX = (img.shape[1] - textsize[0]) // 2
            textY = (img.shape[0] + textsize[1]) // 2
            cv2.putText(img, text, (textX, textY), font, font_scale, (255, 255, 255), thickness=thickness)
            return img

        cv2.namedWindow('', flags=cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        t = None
        cur_img = None
        cur_y = 1
        cur_focus = .5
        cur_pressure = 0
        theta = 0
        a_x = 1 - self.delta.value
        a_y = (1 - self.delta.value) / 2
        h, w = self.figs['normal']['plain'].shape[:2]
        squeezer = Squeezer(h, w)
        delta_x = np.round(self.delta.value * w).astype(int)
        delta_y = np.round(self.delta.value * h).astype(int)
        while True:
            if any(np.equal([self.alert, self.comfort, self.stage, self.num, self.recovery, self.tgt_focus, self.tgt_pressure], None)):
                continue
            if self.reset:
                cur_img = None
                self.reset = False
            if self.stage < 3:
                t_now = time()
                dt = 0 if t is None else t_now - t
                t = t_now
                if not self.stage:
                    tgt_img = self.figs['low']['plain']
                    cur_focus += (.5 - cur_focus) * self.alpha * dt
                    cur_y += (1 - cur_y) * self.alpha * dt
                elif self.stage == 1:
                    if self.alert < 30:
                        stance = 'low'
                    elif self.alert < 70:
                        stance = 'normal'
                    else:
                        stance = 'high'
                    if self.comfort < 40:
                        idx = np.round(100 - self.comfort / 4 * 10).astype(int)
                        tgt_img = self.figs[stance]['unsat'][idx - 1] if idx else self.figs[stance]['plain']
                        cur_focus += (.5 - cur_focus) * self.alpha * dt
                        cur_y += (1 - cur_y) * self.alpha * dt
                    else:
                        idx = np.round((self.comfort - 40) / 6 * 10).astype(int)
                        tgt_img = self.figs[stance]['sat'][idx - 1] if idx else self.figs[stance]['plain']
                        cur_focus += (self.tgt_focus - cur_focus) * self.alpha * dt
                        theta += dt
                        tgt_y = 1 - np.sin(np.pi * 2 / self.period.value * theta) * self.alert / 100
                        cur_y += (tgt_y - cur_y) * self.alpha * dt
                else:
                    idx = np.round(100 - self.recovery).astype(int)
                    tgt_img = self.figs['low']['rot'][idx - 1] if idx else self.figs['low']['plain']
                    cur_focus += (.5 - cur_focus) * self.alpha * dt
                    cur_y += (1 - cur_y) * self.alpha * dt
                cur_pressure += (self.tgt_pressure - cur_pressure) * self.alpha * dt
                tgt_img = squeezer.squeeze(tgt_img, cur_pressure)
                if not self.stage:
                    tgt_img = pixelate(tgt_img, np.round(100 - self.alert / 3 * 10).astype(int))
                if cur_img is None:
                    cur_img = np.array(tgt_img, dtype=float)
                else:
                    cur_img += (tgt_img - cur_img) * self.alpha * dt
                x0 = np.round(a_x * w * (1 - cur_focus)).astype(int)
                y0 = np.round(a_y * h * cur_y).astype(int)
                img = np.full((self.h, self.w, 3), 0, dtype=np.uint8)
                img[(self.h - self.l) // 2:(self.h + self.l) // 2, (self.w - self.l) // 2:(self.w + self.l) // 2] = cv2.resize(cur_img[y0:y0 + delta_y, x0:x0 + delta_x], (self.l, self.l))
                cv2.imshow('', img)
            else:
                img = np.full((self.h, self.w), 0, dtype=np.uint8)
                write_text(img, str(self.num))
                cv2.imshow('', img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    infant_visualizer = InfantVisualizer(1440, 2560)
    Thread(target=infant_visualizer.start_plotting).start()
    rclpy.spin(infant_visualizer)
