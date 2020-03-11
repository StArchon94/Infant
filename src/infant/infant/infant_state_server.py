import os
from time import time

import numpy as np
import rclpy
from infant_interfaces.msg import InfantState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64


class InfantStateServer(Node):
    def __init__(self):
        super().__init__('infant_state_server')
        pd_read_only = ParameterDescriptor(read_only=True)
        # Infant stages:
        # 0 - premature
        # 1 - mature
        # 2 - crying
        # 3,4 - numbering
        self.n_infants_path, self.comfort, self.stage, self.recovery = self.declare_parameters('', [('n_infants_path', '/home/slin/infant_ws/src/infant/resource/n_infants.npy', pd_read_only), ('comfort', 50.0), ('stage', 3), ('recovery', .0)])
        self.n_infants = self.declare_parameter('n_infants', np.asscalar(np.load(self.n_infants_path.value)) if os.path.exists(self.n_infants_path.value) else 0)
        self.ear_alert = 0
        self.eye_alert = 0
        self.alert = .0
        self.sub_ear_alert = self.create_subscription(Float64, 'ear_alert', self.ear_alert_callback, 1)
        self.sub_eye_alert = self.create_subscription(Float64, 'eye_alert', self.eye_alert_callback, 1)
        self.sub_comfort = self.create_subscription(Float64, 'change_comfort', self.comfort_callback, 10)
        self.sub_recovery = self.create_subscription(Float64, 'change_recovery', self.recovery_callback, 10)
        self.pub_state = self.create_publisher(InfantState, 'state', 1)
        self.pub_reset = self.create_publisher(Empty, 'reset', 1)
        self.timer = self.create_timer(.1, self.timer_callback)
        self.t = time()

    def update_alert(self):
        self.alert = np.clip(self.ear_alert + self.eye_alert, 0, 100)
        if not self.stage.value and self.alert > 30:
            self.stage._value = 1
            self.set_parameters([self.stage])

    def ear_alert_callback(self, msg):
        if self.stage.value < 2:
            self.ear_alert = msg.data
            self.update_alert()

    def eye_alert_callback(self, msg):
        if self.stage.value < 2:
            self.eye_alert = msg.data
            self.update_alert()

    def comfort_callback(self, msg):
        if self.stage.value != 1:
            return
        self.comfort._value = np.clip(self.comfort.value + msg.data, 0, 100)
        params = [self.comfort]
        if not self.comfort.value:
            self.stage._value = 2
            params.append(self.stage)
            self.t = time()
        self.set_parameters(params)

    def reset(self):
        self.alert = .0
        self.ear_alert = 0
        self.eye_alert = 0
        self.comfort._value = 50.0
        self.recovery._value = .0
        self.set_parameters([self.comfort, self.recovery])

    def recovery_callback(self, msg):
        if self.stage.value != 2:
            return
        self.recovery._value = np.clip(self.recovery.value + msg.data, 0, 100)
        if self.recovery.value == 100:
            self.pub_reset.publish(Empty())
            self.reset()
            self.stage._value = 1
            self.set_parameters([self.stage])
        else:
            self.set_parameters([self.recovery])

    def timer_callback(self):
        if self.stage.value == 2:
            elapsed = time() - self.t
            if elapsed > 1000:
                self.stage._value = 3
                self.set_parameters([self.stage])
                self.t = time()
        elif self.stage.value == 3:
            elapsed = time() - self.t
            if elapsed > 5:
                np.save(self.n_infants_path.value, self.n_infants.value)
                self.n_infants._value += 1
                self.stage._value = 4
                self.set_parameters([self.n_infants, self.stage])
                self.t = time()
        elif self.stage.value == 4:
            elapsed = time() - self.t
            if elapsed > 5:
                self.pub_reset.publish(Empty())
                self.reset()
                self.stage._value = 0
                self.set_parameters([self.stage])
        self.pub_state.publish(InfantState(alert=self.alert, comfort=self.comfort.value, stage=self.stage.value, num=self.n_infants.value, recovery=self.recovery.value))


def main():
    rclpy.init()
    infant_state_server = InfantStateServer()
    rclpy.spin(infant_state_server)
