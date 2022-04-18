# -*- coding: utf-8 -*-
# @Time : 2022/3/30 14:44
# @File : airsim_control.py
# @Software: PyCharm
# @Author : @white233

import airsim
import time
import numpy as np
# from detect import *
#
# pre_ctrl_gesture = ''  # 初始手势
# velocity = 8.0  # 速度初始化
# speed_add = 5.0  # 速度增量
# x_pose_add = 1000.0  # 前进增量
# z_pose_add = 1000.0  # 上升增量
#
# client = airsim.MultirotorClient()
# client.enableApiControl(True)  # get control
# client.armDisarm(True)  # unlock
#
# kinematic_state = client.simGetGroundTruthKinematics(vehicle_name='')  # get state
# x, y, z = kinematic_state.position
#
# gesture_list = []
#
#
# def jug_filter(ctrl_gesture):
#     if len(gesture_list) <= 10:
#         gesture_list.append(ctrl_gesture)
#         is_ok = False
#     else:
#         gesture_list.pop(0)
#         gesture_list.append(ctrl_gesture)
#         is_ok = True
#
#     max_gesture = max(gesture_list, key=gesture_list.count)
#     print(max_gesture)
#     return max_gesture, is_ok
#
#
# def classed_jug(ctrl_gesture):
#     global pre_ctrl_gesture, x, y, z, velocity, speed_add, x_pose_add, z_pose_add
#
#     now_ctrl_gesture, IS_OK = jug_filter(ctrl_gesture)
#     if IS_OK:
#         if now_ctrl_gesture == 'raised_fist':
#             client.landAsync()  # land
#             # client.armDisarm(False)
#             # client.enableApiControl(False)
#         elif now_ctrl_gesture == 'ok_hand':
#             client.enableApiControl(True)  # get control
#             client.armDisarm(True)  # unlock
#             client.takeoffAsync()  # takeoff
#         elif now_ctrl_gesture == 'raised_hand':
#             client.hoverAsync()  # keep
#         elif now_ctrl_gesture == 'index_pointing_up':
#             x += x_pose_add
#             client.moveToPositionAsync(x, 0, 0, velocity)  # up
#         elif now_ctrl_gesture == 'victory_hand':
#             velocity += speed_add
#         elif now_ctrl_gesture == 'four_fingers':
#             z -= z_pose_add
#             client.moveToPositionAsync(0, 0, z, velocity)  # up
#         pre_ctrl_gesture = now_ctrl_gesture
#     else:
#         pass
#
#     kinematic_state_now = client.simGetGroundTruthKinematics(vehicle_name='')  # get state
#     now_x, now_y, now_z = kinematic_state_now.position
#
#     print(now_ctrl_gesture)
#     print('now_x=%d, now_y=%d, now_z=%d' % (now_x, now_y, now_z))
#     return 0
#
#
# if __name__ == '__main__':
#     # client = airsim.MultirotorClient()
#     #
#     # client.enableApiControl(True)  # get control
#     # client.armDisarm(True)  # unlock
#     # client.takeoffAsync().join()  # takeoff
#     #
#     # main()
a = np.zeros(3, dtype=np.float32)
a += 1.1
a[2] += 1.1
print(a)