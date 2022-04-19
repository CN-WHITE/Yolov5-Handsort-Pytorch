# -*- coding: utf-8 -*-
# @Time : 2022/4/7 19:38
# @File : drone_keyboard_control.py
# @Software: PyCharm
# @Author : @white233

import airsim
from pynput import keyboard
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
import time


class droneController:
    """
    High level drone controller for manual drone navigation using hand.
    """
    def __init__(self):
        self.acceleration = 3.0
        self.max_speed = 20.0
        self.angular_velocity = 90.0
        self.duration = 0.4
        self.friction = 0.05

        self.desired_velocity = np.zeros(3, dtype=np.float32)

        self._hand_command_mapping = {
            'index_pointing_up':                "forward",
            'raised_fist':                      "backward",
            'hand_with_fingers_splayed':        "turn left",
            'little_finger':                    "turn right",
            'four_fingers':                     "up",
            'ok_hand':                          "down",
            'raised_hand':                      "left",
            'sign_of_the_horns':                "right",
            
            'thumbup':                          "add speed",
            'victory_hand':                     "cut speed",
        }

        self._active_commands = {command: False for command in self._hand_command_mapping.values()}

        self._client = airsim.MultirotorClient()
        self._client.confirmConnection()
        self._client.enableApiControl(True)
        self._client.takeoffAsync()

    def move(self, velocity, yaw_rate):
        self._client.moveByVelocityAsync(velocity[0].item(), velocity[1].item(), velocity[2].item(), self.duration,
                                         drivetrain=airsim.DrivetrainType.ForwardOnly,
                                         yaw_mode=airsim.YawMode(True, yaw_rate))

    def fly_by_hand(self, gesture=''):
        """
        Begin to listen for hand and send according control commands until `esc` is pressed.
        """
        print("now:%s" % gesture)
        self._hand_start(gesture)
        self._handle_commands()
        self._hand_end(gesture)

    def _hand_start(self, key):
        if key in self._hand_command_mapping.keys():
            self._active_commands[self._hand_command_mapping[key]] = True
        elif key == keyboard.Key.esc:
            # Shutdown.
            return False

    def _hand_end(self, key):
        if key in self._hand_command_mapping.keys():
            self._active_commands[self._hand_command_mapping[key]] = False

    def _handle_commands(self):
        drone_orientation = ScipyRotation.from_quat(self._client.simGetVehiclePose().orientation.to_numpy_array())
        yaw = drone_orientation.as_euler('zyx')[0]   # 偏航角
        forward_direction = np.array([np.cos(yaw), np.sin(yaw), 0])     # 前后方向
        left_direction = np.array([np.cos(yaw - np.deg2rad(90)), np.sin(yaw - np.deg2rad(90)), 0])  # 左右方向

        # 若前后运动
        if self._active_commands["forward"] or self._active_commands["backward"]:
            forward_increment = forward_direction * self.duration * self.acceleration   # delta_v =  at
            if self._active_commands["forward"]:
                self.desired_velocity += forward_increment  # v = delta_v + at
            else:
                self.desired_velocity -= forward_increment
        elif self._active_commands["cut speed"]:
            forward_component = np.dot(self.desired_velocity, forward_direction) * forward_direction    # 速度的正向分量
            self.desired_velocity -= self.friction * forward_component  # 减速系数
        elif self._active_commands["add speed"]:
            forward_component = np.dot(self.desired_velocity, forward_direction) * forward_direction    # 速度的正向分量
            self.desired_velocity += self.friction * forward_component  # 减速系数

        # 若上下运动
        if self._active_commands["up"] or self._active_commands["down"]:
            vertical_component = drone_orientation.apply(np.array([0.0, 0.0, -1.0]))    # 获得垂直方向速度分量
            vertical_component *= self.duration * self.acceleration     # 垂直分量
            if self._active_commands["up"]:
                self.desired_velocity += vertical_component
            else:
                self.desired_velocity -= vertical_component
        else:
            self.desired_velocity[2] *= self.friction
            pass

        # 若左右运动
        if self._active_commands["left"] or self._active_commands["right"]:
            lateral_increment = left_direction * self.duration * self.acceleration
            if self._active_commands["left"]:
                self.desired_velocity += lateral_increment
            else:
                self.desired_velocity -= lateral_increment
        else:
            left_component = np.dot(self.desired_velocity, left_direction) * left_direction
            self.desired_velocity -= self.friction * left_component

        speed = np.linalg.norm(self.desired_velocity)   # 求速度大小
        if speed > self.max_speed:
            self.desired_velocity = self.desired_velocity / speed * self.max_speed

        yaw_rate = 0.0
        if self._active_commands["turn left"]:
            yaw_rate = -self.angular_velocity
        elif self._active_commands["turn right"]:
            yaw_rate = self.angular_velocity

        self.move(self.desired_velocity, yaw_rate)


if __name__ == "__main__":
    controller = droneController()
    controller.fly_by_hand()

