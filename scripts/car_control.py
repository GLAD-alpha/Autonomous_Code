#!/usr/bin/env python3

# Copyright (c) 2023, Gwangju Institute of Science and Technology
# All rights reserved.

# This code was developed by Hong-Seung Kim, Ji-Kang Kong, and Taehyung Gil as part of their
# work as interns and graduate students at the GLAD lab, Gwangju Institute of Science and
# Technology, Gwangju, South Korea.

# Permission is hereby granted, free of charge, to any person obtaining a copy of this code
# and associated documentation files (the "Code"), to use the Code for academic purposes only,
# subject to the following conditions:

# The Code may only be used for academic, non-commercial purposes.
# Any commercial use of the Code or any part thereof is strictly prohibited, and requires
# a separate, written license from the GLAD lab.
# Redistributions of source code must retain the above copyright notice, this list of
# conditions, and the following disclaimer.

# For any commercial use or licensing inquiries, please contact:
# Professor Yong-Gu Lee
# GLAD Lab, Gwangju Institute of Science and Technology
# Address: Mechanical Engineering #210, 123 Cheomdangwagi-ro,  Buk-gu, Gwangju, 61005, Korea
# Email: lygu@gist.ac.kr

# THE CODE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE CODE OR THE USE OR OTHER DEALINGS IN THE CODE.

from pickle import FALSE
import rospy

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from math import radians
from collections import deque
from can_module import CAN
#import actionlib

#roslib.load_manifest('can_cmd')
#import roslib

from std_msgs.msg import String
from geopy import distance
from attrdict import AttrDict

from PIDController import PIDLongitudinalController
from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL, HEADING2

conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'},
        {'topic':'/novatel/oem7/bestvel', 'type': BESTVEL, 'name': 'velocity'},
        {'topic':'/novatel/oem7/heading2', 'type': HEADING2, 'name': 'yaw'},
        {'topic':'/yolodetection', 'type' : String, 'name' : 'getyolo'}
    ]})

# change parameters here  # 5
param = AttrDict({'max_speed': 5, 'max_accel': 0.5, 'destination': (35.22516886, 126.83953094),
                  'normal_pid_gain':{'p':15, 'i': 0, 'd':2},
                  'back_pid_gain':{'p':15, 'i': 0, 'd':0},
                  'exp_pid_gain':{'p':45, 'i': 0, 'd':0}, # p 40 good, 50 급정지, d = 0이 기본, 7도 좋음
                  'dt': 0.02,
                  'lateral':{'K':0.5},
                  'drivemode' : {'N':1,'B':2,'E':3} # N : {Normal}, B : {Back}, X : {Exception}
                  })

lat = 0
lon = 0 
acc_x = 0
cur_vel = 0 
yaw = 0

gnss_cnt = 0

NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7

class Controller:
    def __init__(self, conf=None, param=None):
        rospy.init_node('controller', anonymous=True)

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback,
                          '/novatel/oem7/heading2': self.yaw_callback,
                          '/yolodetection' : self.yolo_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.imu_rate = 100
        self.pub_rate = 10
        self.param = param

        self.max_speed = param.max_speed
        self.max_accel = param.max_accel

        self.lon_controller = PIDLongitudinalController(param)

        self.can = CAN()
        print('hiyo')
        
        # Driving Flag
        self.drive_flag = True
        self.gnss_start_flag = True
        self.stop_flag = False

        while self.gnss_start_flag == True:
            pass
        self.start_pos = (lat, lon)
        
        #self.start_pos = (35.22510137, 126.83975056)
        #self.target_pos = (35.22545213, 126.83870505)
        print("start pos : ",self.start_pos)
        self.target_pos = (35.2251838203,126.839521686)
        self.max_travel_distance = distance.distance(self.start_pos, self.target_pos)
        print("max distance : ",self.max_travel_distance)
        self.path = []
        self.index = 0
        self.initial_gear()

        self.cur_vel_log = []
        self.cmd_vel_log = []
        self.break_log = []

    def initial_gear(self):
        #self.can.get_feedback()
        print("********Initial_gear***********")
        while self.can.info_1_dict["Gear_shift_Feedback"] == 0:
            self.can.get_feedback()
            print('off')
            #print(self.can.info_1_dict["Gear_shift_Feedback"])
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Parking":
            self.can.change_gear(REVERSE)
            print('11') 
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Reverse":
            self.can.change_gear(NEUTRAL)
            print('22')        
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Neutral":
            self.can.change_gear(DRIVE)
            print('33')
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Driving":
            print('Current Driving')
        
        print("Initial Gear")

    def pos_callback(self, data):
        global lat, lon, gnss_cnt
        if (data.pos_type.type == 56 or data.pos_type.type == 50 or data.pos_type.type == 34) and self.gnss_start_flag:
            gnss_cnt += 1
            print(gnss_cnt)
            if gnss_cnt >= 10:
                lat = data.lat
                lon = data.lon
                gnss_cnt = 0
                self.gnss_start_flag = False
        else:
            lat = data.lat
            lon = data.lon
        # 56 : INS
        # 50 : Narrow_INT    

    def yolo_callback(self, data):
        global yolo_data
        if data.data == 'stop':
            self.drive_flag = False
            #print('stop')
        elif data.data == 'go':
            self.drive_flag = True
            #print('go')
    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc / self.imu_rate

    def spd_callback(self, data):
        global cur_vel
        cur_vel = data.hor_speed 
    
    def yaw_callback(self, data):
        # 값 개튐
        global yaw
        yaw = data.heading

    def get_target_velocity(self):
        dist = float(distance.distance((lat, lon), self.start_pos).km)
        tar_dist = float(distance.distance((lat, lon), self.target_pos).km)
        if dist > self.max_travel_distance:
            return 0
        if dist * 1000 < 0.5 * self.max_speed ** 2 / self.max_accel:
            return sqrt(2 * self.max_accel * dist * 1000)
        if tar_dist * 1000 < 0.5 * self.max_speed ** 2 / self.max_accel:
            return sqrt(2 * self.max_accel * abs(tar_dist) * 1000)
        else:
            return self.max_speed
        

    def run_step(self):
        if self.drive_flag == False:
            # 정지 수신호 들어옴. 멈춰야함
            #self.can.get_feedback()
            #print("Speed : ",self.can.info_2_dict["Vehicle_Speed"])
            #while self.can.info_2_dict["Vehicle_Speed"] >= 1:
                # tar_vel = 0 으로 => 감속
            #print('hihihihihi')
            cmd_acc = self.lon_controller.run_step(cur_vel, -1.0, self.param.drivemode.E)
            cmd_steer = 0
            self.send_can_control(cmd_acc, cmd_steer)
            self.break_log.append(cur_vel)
            #self.can.get_feedback()
        # 속도가 0 이면
            self.stop_flag = True

        elif self.drive_flag == True:
            if self.stop_flag:
                print("restart driving")
                # 정지 수신호에 의해 멈춘 이후 다시 주행 신호왔을 때
                self.start_pos = (lat,lon)
                self.stop_flag = False
            else:
                # 일반 주행
                tar_vel = self.get_target_velocity()
                cmd_acc = self.lon_controller.run_step(cur_vel, tar_vel, self.param.drivemode.N)
                cmd_steer = 0

                self.send_can_control(cmd_acc, cmd_steer)
                
                self.cur_vel_log.append(cur_vel)
                self.cmd_vel_log.append(tar_vel)
        



    def send_can_control(self, acc, steer):
        if acc > 0:
            self.can.driving_cmd_dict["Brake_CMD"] = 0
            self.can.driving_cmd_dict["Accel_CMD"] = 650 + int(acc * 55)
        elif acc > -0.2:
            self.can.driving_cmd_dict["Accel_CMD"] = 650
            self.can.driving_cmd_dict["Brake_CMD"] = 0 
        else:
            self.can.driving_cmd_dict["Accel_CMD"] = 650
            self.can.driving_cmd_dict["Brake_CMD"] = min(int(-acc * 1000),13000)
        self.can.send_control()

    def reached_destination(self):
        #print(self.index, len(self.path))
        if distance.distance(self.start_pos, (lat, lon)) > self.max_travel_distance:
            print("1_reach")
            return True
        if distance.distance((lat, lon), self.target_pos).km > 0.003:
            return False
        else:
            print("reached destination")
            return True

    def make_plot(self):
        plt.figure(1)
        plt.plot(list(range(len(self.cur_vel_log))), self.cur_vel_log)
        plt.plot(list(range(len(self.cmd_vel_log))), self.cmd_vel_log)
        plt.show()
        plt.savefig('./go.png', dpi=300)
        plt.figure(2)
        plt.plot(list(range(len(self.break_log))), self.break_log)
        plt.show()
        plt.savefig('./break.png', dpi=300)
        
if __name__ == '__main__':
    try:
        vehicle = Controller(conf, param)
        rate = rospy.Rate(50)
        while not vehicle.reached_destination() and not rospy.is_shutdown():
            vehicle.run_step()
            rate.sleep()
        else:
            print("now braking")
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Driving":
                print('Current Driving')
                vehicle.can.change_gear(NEUTRAL)
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Neutral":
                vehicle.can.change_gear(REVERSE)
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Reverse":
                vehicle.can.change_gear(PARKING)
        print("Done")
        vehicle.make_plot()
    except KeyboardInterrupt:
        vehicle.send_can_control(-11, 0)
        print("end control")
