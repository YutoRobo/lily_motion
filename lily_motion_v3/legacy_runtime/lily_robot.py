#!/usr/bin/env python
# coding: utf-8
from turtle import pos
try:
    import rospy
except Exception:
    rospy = None
import numpy as np
import math
from sympy import Matrix
try:
    from shapely.geometry import Point, multipoint, LineString
except Exception:
    Point = multipoint = LineString = None
from lily_motion_v3.legacy_runtime import servo
from lily_motion_v3.legacy_runtime import leg, util
from lily_motion_v3.legacy_runtime.util import Posture

# ロボットの軌跡追従及び脚先座標算出のレイヤー
class LilyRobot:
    def __init__(self, endEfectorManagers):
        self.endEfectorManagers = endEfectorManagers
        # 現在のロボット重心の姿勢
        self.posture = Posture()
        # 胴体の幅
        self.__body = 0.3
        # 基準姿勢での支持四角形の幅
        self.__default_support_dist = 0.8
        # 床の方向 U:上, B:下, F:前, H:後, L:左, R:右
        self.__groud_vector = "B"
        # 支持脚である脚の番号  0:BLF, 1:BLH, 2:BRF, 3:BRH, 4:TLF, 5:TLH, 6:TRF, 7:TRH
        self.__support_leg = [0, 1, 2, 3]
        self.__landing_leg = []
        # 2種の解析解の内どれか。
        self.__support_solve_type = [0,0,0,0,0,0,0,0]
        #self.__support_solve_type = [0,0,2,2,0,0,0,0]
        self.__lending_leg_type = []
        self.__swing_leg_type = []
        # 脚先の座標（たぶんロボット座標系）
        self.__end_effector_position = [
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]]),
            Matrix([[0], [0], [0]])
        ]

        self.__max_landing_step = 0
        self.__now_landing_step = 0
        self.__max_swing_step = 0
        self.__now_swing_step = 0
    
    # ロボット姿勢の設定(初期設定)
    def setPosture(self, posture):
        self.posture = posture
    
    # ロボット情報の設定(ただしロボット寸法を変えるとLeg.pyの計算と辻褄が合わなくなるので注意)
    def setRobotParam(self, body):
        self.__body = body
    
    # 支持脚を設定
    def setSupportLeg(self, support_leg):
        self.__support_leg = support_leg
    
    def setDefaultSupportDist(self, default_support_dist):
        self.__default_support_dist = default_support_dist

    # 解選択の重み設定
    def setWeight(self, weight_1, weight_2, weight_3):
        for i in range(8):
            self.endEfectorManagers[i].setWeight(weight_1, weight_2, weight_3)

    
    # 基準姿勢に強制的に戻すコマンド
    # Args: groud_dist:ロボットのどの方向が床面であるか
    def setDefaultPose(self, max_step):
        if self.__groud_vector == "B":
            self.__end_effector_position[0] = Matrix([[self.__default_support_dist/2.0], [self.__default_support_dist/2.0], [-self.posture.z]])
            self.__end_effector_position[1] = Matrix([[-self.__default_support_dist/2.0], [self.__default_support_dist/2.0], [-self.posture.z]])
            self.__end_effector_position[2] = Matrix([[self.__default_support_dist/2.0], [-self.__default_support_dist/2.0], [-self.posture.z]])
            self.__end_effector_position[3] = Matrix([[-self.__default_support_dist/2.0], [-self.__default_support_dist/2.0], [-self.posture.z]])
            self.__end_effector_position[4] = Matrix([[self.__default_support_dist/2.0], [self.__default_support_dist/2.0], [self.posture.z]])
            self.__end_effector_position[5] = Matrix([[-self.__default_support_dist/2.0], [self.__default_support_dist/2.0], [self.posture.z]])
            self.__end_effector_position[6] = Matrix([[self.__default_support_dist/2.0], [-self.__default_support_dist/2.0], [self.posture.z]])
            self.__end_effector_position[7] = Matrix([[-self.__default_support_dist/2.0], [-self.__default_support_dist/2.0], [self.posture.z]])
        for i in range(8):
            self.endEfectorManagers[i].setTargetAbsolutePose(self.__end_effector_position[i], max_step)

    # 支持脚の動作設定
    def setSupportMove(self, posture_diff, max_step=1,support_solve_type = None):
        #self.__max_support_step = max_step
        self.__posture_step_diff = posture_diff.div(float(max_step))
        if support_solve_type is not None:
            self.__support_solve_type = support_solve_type
        # 開始時に各脚の絶対座標を取得
        self.__start_position = [util.TransformationRobotToABS(self.endEfectorManagers[leg].getEndEffectorPose(),self.posture) for leg in range(8)]

        # TODO:支持脚に対して、第二関節が床にめり込んでいるか判定⇒適切な姿勢を決定。
        '''        
        for i in self.__support_leg:
            # 現在のエンドエフェクタ座標を取得
            end_effector_position_tmp = self.__end_effector_position[i]
            # 各姿勢が各STEPで床にめり込んでいないかどうかのログ
            list_not_overgroud_type = [[],[],[],[]]
            # 各姿勢で事前シミュレーション
            for step in range(max_step):
                # 脚先座標目標値を更新
                end_effector_position_tmp = util.TransformationAroundOrigin(end_effector_position_tmp, self.__posture_step_diff.reverse())
                self.endEfectorManagers[i].setTargetAbsolutePose(end_effector_position_tmp, max_step=1) 
                pos = self.endEfectorManagers[i].calcAnalyticalInverseKinematics(self.__support_solve_type[0], update_servo=False, get_pos=True)
                # print(pos)
                # TODO:第二関節の座標に応じて成否を場合分けしてlist_not_overgroud_typeに登録
        '''
    
    # 遊脚の動作
    # 目標座標は絶対座標
    def setLandingMove(self, target_position, landing_leg, lending_leg_type, max_step, weight = None):
        # TODO:次に着地する2脚を推定し選択
        self.__max_landing_step = max_step
        self.__now_landing_step = 0
        self.__landing_leg_position = target_position
        self.__lending_leg_type = lending_leg_type
        self.__landing_leg = landing_leg

        if weight is not None:
            for i in range(len(self.__landing_leg)):
                leg = self.__landing_leg[i]
                self.endEfectorManagers[leg].setWeight(weight[0], weight[1], weight[2])


    # 指定した脚を指定した回転角に変位させる設定
    def setSwingLeg(self, swing_leg, swing_leg_type, max_step):
        self.__swing_leg = swing_leg
        self.__now_swing_step = 0
        self.__swing_leg_type = swing_leg_type
        self.__max_swing_step = max_step
        #print(max_step)

        for i in range(len(self.__swing_leg)):
            leg = self.__swing_leg[i]
            DEG2 = 60
            DEG3 = 120
            if self.__swing_leg_type[i] == 0:
                self.endEfectorManagers[leg].setTargetDegree([[0], [DEG2], [-DEG3]], max_step)
            elif self.__swing_leg_type[i] == 1:
                self.endEfectorManagers[leg].setTargetDegree([[0], [-DEG2], [DEG3]], max_step)
            elif self.__swing_leg_type[i] == 3:
                self.endEfectorManagers[leg].setTargetDegree([[-180], [DEG2], [-DEG3]], max_step)
            elif self.__swing_leg_type[i] == 4:
                self.endEfectorManagers[leg].setTargetDegree([[180], [DEG2], [-DEG3]], max_step)
            elif self.__swing_leg_type[i] == 5:
                self.endEfectorManagers[leg].setTargetDegree([[180], [-DEG2], [DEG3]], max_step)
            elif self.__swing_leg_type[i] == 6:
                self.endEfectorManagers[leg].setTargetDegree([[-180], [-DEG2], [DEG3]], max_step)
            elif self.__swing_leg_type[i] == -1:
                now_deg = self.endEfectorManagers[leg].leg.getServosDeg()
                print("Set Swing")
                if now_deg[0][0] >= 180:
                    print("over 180")
                    self.endEfectorManagers[leg].setTargetDegree([[now_deg[0][0]-180], [now_deg[1][0]], [now_deg[2][0]]], max_step)
                elif now_deg[0][0] <= -180:
                    print("under -180")
                    self.endEfectorManagers[leg].setTargetDegree([[now_deg[0][0]+180], [now_deg[1][0]], [now_deg[2][0]]], max_step)
                else:
                    print(now_deg)

    # TODO:支持脚と遊脚を判断するフェーズ
    def judgeSupprotAndSwing():
        pass
        
    # 姿勢の変化量を入力
    # 支持脚の相対的な位置関係を固定したまま移動可能
    def suportMove(self):
        # ロボットの姿勢を更新
        # TODO:遊脚⇒支持脚の切り替えタイミングでロボット姿勢と支持脚座標の帳尻合わせ処理が必要。
        self.posture = self.posture + self.__posture_step_diff
        
        # 支持脚のロボット座標の計算
        for i in self.__support_leg:
            # 座標変換　ロボット座標系上で微小回転
            #self.__end_effector_position[i] = util.TransformationAroundOrigin(self.__end_effector_position[i], self.__posture_step_diff.reverse()) # 注意：脚先座標の微小回転を繰り返すと、桁落ちによりロボットの姿勢と数cm程度誤差が生じてくる。
            self.__end_effector_position[i] = util.TransformationAroundOrigin(self.__start_position[i], self.posture.reverse())
            self.endEfectorManagers[i].setTargetAbsolutePose(self.__end_effector_position[i], max_step=1) 
    
    # TODO
    def landingMove(self):
        if self.__now_landing_step < self.__max_landing_step:
            self.__now_landing_step = self.__now_landing_step + 1
            if len(self.__landing_leg) > 0:
                for i in range(len(self.__landing_leg)):
                    leg = self.__landing_leg[i]
                    # TODO:現在姿勢に基づき脚先目標座標をロボット座標に変更
                    # 若干挙動おかしいから注意
                    tar_p = self.__start_position[leg]+(self.__landing_leg_position[i]-self.__start_position[leg])*float(self.__now_landing_step)/float(self.__max_landing_step)
                    self.__end_effector_position[leg] = util.TransformationABSToRobot(tar_p, self.posture)
                    self.endEfectorManagers[leg].setTargetAbsolutePose(self.__end_effector_position[leg], max_step=1) 
                    #print(tar_p)
        
        if self.__now_landing_step >= self.__max_landing_step:
            #print("reset landing")
            self.__landing_leg_position = []
            self.__lending_leg_type = []
            self.__landing_leg = []
            

    def calcInverseKinematics(self, use_all_leg=False):
        # 支持脚を動かす。
        if use_all_leg:
            for i in range(8):
                self.endEfectorManagers[i].calcInverseKinematics(update_servo=True)
        else:
            for i in self.__support_leg:
                self.endEfectorManagers[i].calcInverseKinematics(update_servo=True)
    
    def calcAnalyticalInverseKinematics(self):
        # 支持脚を動かす。
        for i in self.__support_leg:
            self.endEfectorManagers[i].calcAnalyticalInverseKinematics(self.__support_solve_type[i], self.posture, update_servo=True)
        # 着地脚を動かす。
        if len(self.__landing_leg) > 0:
            for i in range(len(self.__landing_leg)):
                leg = self.__landing_leg[i]
                self.endEfectorManagers[leg].calcAnalyticalInverseKinematics(self.__lending_leg_type[i], self.posture, update_servo=True)
        
    def swingMove(self):
        if self.__now_swing_step < self.__max_swing_step:
            self.__now_swing_step = self.__now_swing_step + 1
            if len(self.__swing_leg) > 0:
                for i in range(len(self.__swing_leg)):
                    leg = self.__swing_leg[i]
                    self.endEfectorManagers[leg].calcTargetDegree()
        
        if self.__now_swing_step >= self.__max_swing_step:
            #print("reset swing")
            self.__swing_leg_type = []
            self.__swing_leg = []
    
    def toggleDirectionAtRolling(self):
        for i in range(8):
            self.endEfectorManagers[i].leg.toggleDirectionAtRolling()



