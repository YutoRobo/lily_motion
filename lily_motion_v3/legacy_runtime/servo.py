#!/usr/bin/env python
# coding: utf-8
import math
import traceback

MIN_DEGREE = -180.0
MAX_DEGREE = 180.0

class ServoMotor:
    def __init__(self, target_deg = 0.0001):
        self.__target_deg = target_deg
        self.__min_deg = MIN_DEGREE
        self.__max_deg = MAX_DEGREE
        self.__offset_deg = 0.0

    def setOffsetDegree(self, offset_deg):
        self.__offset_deg = offset_deg

    def setInitTargetDegree(self, target_deg):
        self.__target_deg = target_deg

    def setTargetDegree(self, tar_deg):
        tar_deg = tar_deg + self.__offset_deg
        if tar_deg <= self.__min_deg:
            print("target deg min", self.__min_deg, tar_deg)
            self.__target_deg  = self.__min_deg
            raise ValueError("target deg min")
        elif tar_deg >= self.__max_deg:
            print("target deg max", self.__max_deg, tar_deg)
            self.__target_deg  = self.__max_deg
            raise ValueError("target deg max")
        else:
            self.__target_deg  = tar_deg
        # print("set target degree: ", self.__target_deg, " rad: ", math.radians(self.__target_deg))

    def setTargetTheta(self, tar_theta):
        self.setTargetDegree(tar_theta * 180.0 /3.14159)
    
    def setDegreeRange(self, min_deg, max_deg):
        self.__min_deg = min_deg
        self.__max_deg = max_deg

    def getTargetDegree(self):
        return self.__target_deg

    def getTargetTheta(self):
        return self.__target_deg * 3.14159 / 180.0
        
    def send(self):
        print("move start to ", self.__target_deg)
        ## 以下にmbedへの送信の処理を記述


    