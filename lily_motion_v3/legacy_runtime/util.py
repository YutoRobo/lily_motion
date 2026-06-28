#!/usr/bin/env python
# coding: utf-8
import numpy as np
import math
from sympy import Matrix

# 姿勢
class Posture:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = x, y, z, roll, pitch, yaw

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        roll = self.roll + other.roll
        pitch = self.pitch + other.pitch
        yaw = self.yaw + other.yaw
        return Posture(x,y,z,roll,pitch,yaw)
    
    def div(self, other):
        x = self.x / other
        y = self.y / other
        z = self.z / other
        roll = self.roll / other
        pitch = self.pitch / other
        yaw = self.yaw / other
        return Posture(x,y,z,roll,pitch,yaw)

    def reverse(self):
        return Posture(-self.x,-self.y,-self.z,-self.roll,-self.pitch,-self.yaw)

# 原点回りに平行移動＋回転を行った際の座標変換
# 注意：現在はpitch回転のみ
def TransformationAroundOrigin(input_position, diff_posture):
    return_position = input_position + Matrix([[ diff_posture.x ], [ diff_posture.y ], [ diff_posture.z ]])
    # 回転変換
    # 注意：現在はpitch回転のみ
    Ry = Matrix([
        [math.cos(diff_posture.pitch), 0, math.sin(diff_posture.pitch)],
        [0, 1, 0],
        [-math.sin(diff_posture.pitch), 0, math.cos(diff_posture.pitch)]
        ])
    return_position = Ry * return_position
    return return_position

# 座標変換　ロボット系⇒絶対座標系
# 注意：現在はpitch回転のみ
def TransformationRobotToABS(input_position, now_posture):
    posture = now_posture
    # 回転変換
    # 注意：現在はpitch回転のみ
    Ry = Matrix([
        [math.cos(posture.pitch), 0, math.sin(posture.pitch)],
        [0, 1, 0],
        [-math.sin(posture.pitch), 0, math.cos(posture.pitch)]
        ])
    return_position = Ry * input_position + Matrix([[ posture.x ], [ posture.y ], [ posture.z ]])
    return return_position

# 座標変換　絶対座標系⇒ロボット座標系
# 注意：現在はpitch回転のみ
def TransformationABSToRobot(input_position, now_posture):
    posture = now_posture.reverse()
    return_position = input_position + Matrix([[ posture.x ], [ posture.y ], [ posture.z ]])
    # 回転変換
    # 注意：現在はpitch回転のみ
    Ry = Matrix([
        [math.cos(posture.pitch), 0, math.sin(posture.pitch)],
        [0, 1, 0],
        [-math.sin(posture.pitch), 0, math.cos(posture.pitch)]
        ])
    return_position = Ry * return_position
    return return_position