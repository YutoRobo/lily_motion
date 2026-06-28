#!/usr/bin/env python
# coding: utf-8
from sympy import Matrix
import numpy as np
import math
from lily_motion_v3.legacy_runtime import leg

class EndEfectorManager:
    # Args: {leg:脚オブジェクト}, {connection_coordinates:節接続座標P}, {is_left:左脚か否か}
    def __init__(self, leg):
        self.leg = leg
        self.__pos_list = []
        self.__deg_list = []
    
    def setTargetAbsolutePose(self, abs_pose, max_step=1):
        self.__pos_list = []
        start_pose = self.leg.getEndEffectorPose()
        for step in range(max_step):
            self.__pos_list.append((abs_pose - start_pose) * (float(step + 1) /  float(max_step)) + start_pose)
    
    def setTargetRelativePose(self, relative_pose, max_step=1):
        start_pose = self.leg.getEndEffectorPose()
        target_pose = start_pose + relative_pose
        self.__pos_list = []
        for step in range(max_step):
            self.__pos_list.append((target_pose - start_pose) * (float(step + 1) /  float(max_step)) + start_pose)
    
    def setTargetDegree(self, target_degree, max_step=1):
        start_deg = self.leg.getServosDeg()
        '''
        print("start_deg")
        print(start_deg)
        print("target_degree")
        print(target_degree)
        '''
        self.__deg_list = []
        for step in range(max_step):
            self.__deg_list.append((target_degree-start_deg) * (float(step + 1) /  float(max_step)) + start_deg)
        #print(self.__deg_list)

    def setWeight(self, weight_1, weight_2, weight_3):
        self.leg.setWeight(weight_1, weight_2, weight_3)

    def calcInverseKinematics(self, update_servo=True):
        #print("now last", len(self.__pos_list))
        if len(self.__pos_list) > 0:
            self.leg.calcInverse(self.__pos_list[0] - self.leg.getEndEffectorPose(), update_servo=update_servo)
            self.__pos_list.pop(0)
    
    def calcAnalyticalInverseKinematics(self, solve_type, posture, update_servo=True, get_pos=False):
        #print("now last", len(self.__pos_list))
        if len(self.__pos_list) > 0:
            return_pos = self.leg.calcAnalyticalInverse(self.__pos_list[0], solve_type, posture, update_servo=update_servo, get_pos=get_pos)
            self.__pos_list.pop(0)
            if get_pos == True:
                return return_pos
    
    def calcTargetDegree(self):
        if len(self.__deg_list) > 0:
            #print(self.__deg_list[0])
            self.leg.setTargetDegree(self.__deg_list[0])
            self.__deg_list.pop(0)

    def getEndEffectorPose(self):
        return self.leg.getEndEffectorPose()

    def getEndEffectorManipulability(self):
        return self.leg.getEndEffectorManipulability()

    def convertPoseCoorinateLegToRobot(self, pose):
        return [-pose[1]+self.__connection_point_abs[0],pose[0]+self.__connection_point_abs[1],pose[2]] if self.__is_left else [pose[1]+self.__connection_point_abs[0],-pose[0]+self.__connection_point_abs[1],pose[2]] 

    def convertVectorCoorinateLegToRobot(self, vector):
        return [-vector[1],vector[0],vector[2]] if self.__is_left else [vector[1],-vector[0],vector[2]]

    # ヤコビ行列のみを取得
    def getJacobByStep(self):
        return self.leg.getJacobByStep()

if __name__ == '__main__':
    from lily_motion_v3.legacy_runtime import servo
    from lily_motion_v3.legacy_runtime.leg import Omega
    AXIS_1 = Omega(Phi=0, Theta=0, Psi=0)
    AXIS_2 = Omega(Phi=-math.pi/2, Theta=0, Psi=0)
    AXIS_3 = Omega(Phi=0, Theta=0, Psi=0)
    servos = [
        servo.ServoMotor(target_deg=0.0), 
        servo.ServoMotor(target_deg=-45.0), 
        servo.ServoMotor(target_deg=45.0), ]
    leg = leg.Leg(*servos)
    leg.setLinkLength([0, 1.0, 1.0, 1.0])
    leg.setAxis([AXIS_1, AXIS_2, AXIS_3])
    endEfectorManager = EndEfectorManager(leg)

    target_pose_matrix = Matrix([[2.2], [0.0], [-1.0]])
    endEfectorManager.setTatgetPose(target_pose_matrix)
    while(1):
        if endEfectorManager.moveTatgetPose():
            break
