import os
from threading import Thread

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64

from .centroid_tracker import CentroidTracker
from .FaceBoxes.face_detector import FaceDetector


class InfantEye(Node):
    def __init__(self):
        super().__init__('infant_eye')
        self.reset = False
        pd_read_only = ParameterDescriptor(read_only=True)
        self.cam_id, self.tracker_half_life, self.tracker_timeout, self.base_alert = self.declare_parameters('', [('cam_id', 4, pd_read_only), ('tracker_half_life', 30, pd_read_only), ('tracker_timeout', 3, pd_read_only), ('base_alert', 300)])
        
        self.pub_alert = self.create_publisher(Float64, 'eye_alert', 1)
        self.pub_focus = self.create_publisher(Float64, 'eye_focus', 1)
        self.sub_reset = self.create_subscription(Empty, 'reset', self.reset_callback, 1)

    def start_looking(self):
        cap = cv2.VideoCapture(self.cam_id.value)
        img_area = None
        face_detector = FaceDetector(os.path.join(get_package_share_directory('infant'), 'resource/FaceBoxes.pth'))
        ct = CentroidTracker(self.tracker_half_life.value, self.tracker_timeout.value)
        while True:
            if self.reset:
                ct.reset()
                self.reset = False
            _, img = cap.read()
            if not img_area:
                img_w = img.shape[1]
                img_area = np.prod(img.shape[:2])
            faces = ct.update(face_detector.detect_face(img)[:, :4])
            sum_alert = .0
            max_alert = None
            focus = .5
            for centroid, area, weight in faces:
                alert = self.base_alert.value * weight * np.sqrt(area / img_area)
                sum_alert += alert
                if not max_alert or alert > max_alert:
                    focus = 1 - centroid[0] / img_w
                    max_alert = alert
            self.pub_alert.publish(Float64(data=sum_alert))
            self.pub_focus.publish(Float64(data=focus))
        # cap.release()

    def reset_callback(self, msg):
        self.reset = True


def main():
    rclpy.init()
    infant_eye = InfantEye()
    Thread(target=infant_eye.start_looking).start()
    rclpy.spin(infant_eye)
