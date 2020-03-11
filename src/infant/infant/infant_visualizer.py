import argparse
import math
import os
from threading import Thread
from time import time

import cv2
import numpy as np
import rclpy
from infant_interfaces.msg import InfantState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


class InfantVisualizer(Node):
    def __init__(self):
        super().__init__('infant_visualizer')
        self.reset = False
        pd_read_only = ParameterDescriptor(read_only=True)
        self.resource_dir, self.half_life, self.delta_x, self.delta_y, self.period = self.declare_parameters('', [('resource_dir', '/home/slin/infant_ws/src/infant/resource/', pd_read_only), ('half_life', 0.4, pd_read_only), ('delta_x', .85, pd_read_only), ('delta_y', .9, pd_read_only), ('period', 3)])
        self.figs = {}
        self.figs['plain'] = cv2.imread(os.path.join(self.resource_dir.value, 'plain.jpg'))
        self.figs['pixelated'] = [cv2.imread(os.path.join(self.resource_dir.value, f'pixelated/pixelated_{i+1}.jpg')) for i in range(100)]
        self.figs['rotten'] = [cv2.imread(os.path.join(self.resource_dir.value, f'rotten/rotten_{i+1}.jpg')) for i in range(100)]
        self.figs['satisfied'] = [cv2.imread(os.path.join(self.resource_dir.value, f'satisfied/satisfied_{i+1}.jpg')) for i in range(100)]
        self.figs['unsatisfied'] = [cv2.imread(os.path.join(self.resource_dir.value, f'unsatisfied/unsatisfied_{i+1}.jpg')) for i in range(100)]
        self.alpha = np.log(2) / self.half_life.value

        self.sub_state = self.create_subscription(InfantState, 'state', self.state_callback, 1)
        self.sub_focus = self.create_subscription(Float64, 'eye_focus', self.focus_callback, 1)
        self.sub_reset = self.create_subscription(Empty, 'reset', self.reset_callback, 1)
        self.alert = None
        self.comfort = None
        self.stage = None
        self.num = None
        self.recovery = None
        self.tgt_focus = None

    def state_callback(self, msg):
        self.alert = msg.alert
        self.comfort = msg.comfort
        self.stage = msg.stage
        self.num = msg.num
        self.recovery = msg.recovery

    def focus_callback(self, msg):
        self.tgt_focus = msg.data

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
            cv2.putText(img, text, (textX, textY), font, font_scale, (0, 0, 0), thickness=thickness)
            return img

        cv2.namedWindow('', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        t = None
        cur_img = None
        cur_y = 1
        cur_focus = .5
        theta = 0
        a_x = 1 - self.delta_x.value
        a_y = (1 - self.delta_y.value) / 2
        h, w = self.figs['plain'].shape[:2]
        delta_x = np.round(self.delta_x.value * w).astype(int)
        delta_y = np.round(self.delta_y.value * h).astype(int)
        while True:
            if self.alert is None or self.comfort is None or self.stage is None or self.num is None or self.recovery is None or self.tgt_focus is None:
                continue
            if self.reset:
                cur_img = None
                self.reset = False
            if self.stage < 3:
                t_now = time()
                dt = 0 if t is None else t_now - t
                t = t_now
                if not self.stage:
                    id = np.round(100 - self.alert / 3 * 10).astype(int)
                    tgt_img = self.figs['pixelated'][id - 1] if id else self.figs['plain']
                    cur_focus += (.5 - cur_focus) * self.alpha * dt
                    cur_y += (1 - cur_y) * self.alpha * dt
                elif self.stage == 1:
                    if self.comfort < 40:
                        id = np.round(100 - self.comfort / 4 * 10).astype(int)
                        tgt_img = self.figs['unsatisfied'][id - 1] if id else self.figs['plain']
                        cur_focus += (.5 - cur_focus) * self.alpha * dt
                        cur_y += (1 - cur_y) * self.alpha * dt
                    else:
                        id = np.round((self.comfort - 40) / 6 * 10).astype(int)
                        tgt_img = self.figs['satisfied'][id - 1] if id else self.figs['plain']
                        cur_focus += (self.tgt_focus - cur_focus) * self.alpha * dt
                        theta += dt
                        tgt_y = 1 - np.sin(np.pi * 2 / self.period.value * theta) * self.alert / 100
                        cur_y += (tgt_y - cur_y) * self.alpha * dt
                else:
                    id = np.round(100 - self.recovery).astype(int)
                    tgt_img = self.figs['rotten'][id - 1] if id else self.figs['plain']
                    cur_focus += (.5 - cur_focus) * self.alpha * dt
                    cur_y += (1 - cur_y) * self.alpha * dt
                if cur_img is None:
                    cur_img = np.array(tgt_img, dtype=float)
                else:
                    cur_img += (tgt_img - cur_img) * self.alpha * dt
                x0 = np.round(a_x * w * (1 - cur_focus)).astype(int)
                y0 = np.round(a_y * h * cur_y).astype(int)
                cv2.imshow('', cv2.resize(cur_img[y0:y0 + delta_y, x0:x0 + delta_x].astype(np.uint8), (1080, 1920)))
            else:
                img = np.full((1920, 1080), 255, dtype=np.uint8)
                write_text(img, str(self.num))
                cv2.imshow('', img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    infant_visualizer = InfantVisualizer()
    Thread(target=infant_visualizer.start_plotting).start()
    rclpy.spin(infant_visualizer)
