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


import cantools
import can
import threading
from pygame import time
from time import sleep

class GearStateError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


PARKING = 0
DRIVE = 5
NEUTRAL = 6
REVERSE = 7

GEAR_STATE = [PARKING, REVERSE, NEUTRAL, DRIVE]

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

dt = 0.02


class CAN:
    def __init__(self, destination = None):
        # CAN
        # self.bus = can.interfaces.pcan.PcanBus(channel='PCAN1_USBBUS1', bitrate=500000)
        self.db = cantools.database.load_file("/home/nvidia/xavier_workspace/ros/catkin_ws/src/autocar/can_control/DBC/Santafe_Final.dbc")
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

        # self.bus = can.interfaces.pcan.PcanBus(channel='PCAN1_USBBUS1', bitrate=500000)
        self.vehicle_info_1_msg = self.db.get_message_by_name('Vehicle_info_1')
        self.vehicle_info_2_msg = self.db.get_message_by_name('Vehicle_Info_2')
        self.info_1_dict = {"APS_Feedback":0, "Brake_ACT_Feedback":0,"Gear_shift_Feedback":0, 
                               "Steering_angle_Feedback":0, "Switch_state":0}
        self.info_2_dict = {"Override_feedback":0, "Vehicle_Speed":0}
        self.control_cmd_msg = self.db.get_message_by_name('Control_CMD') 
        self.driving_cmd_msg = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_dict = {'Override':0,'Alive_Count':0,'Angular_Speed_CMD':30}  
        self.driving_cmd_dict = {'Accel_CMD':650,'Brake_CMD':0,'Steering_CMD':0,'Gear_shift_CMD':PARKING} 

        self.destination = destination

        self.max_speed = 7  # m/s 
        self.clock = time.Clock()

        self.cur_dist = 0
        self.initialize()

    def initialize(self):
        self.get_feedback()
        # print(self.info_1_dict["Gear_shift_Feedback"])

        # delay = 0
        # while delay < 100:
        #     self.clock.tick_busy_loop(50)
        #     self.get_feedback()
        # if self.info_1_dict["Gear_shift_Feedback"] != "Parking":
        #     print('no')
        #     raise GearStateError("default gear state must be PARKING!")

    def get_feedback(self):
        self.get_vehicle_info_1()
        self.get_vehicle_info_2()

    
    def send_control(self):

        self.send_driving_cmd()
        

    def change_gear(self, gear):
        if gear == NEUTRAL: gear_feed = "Neutral"
        elif gear == DRIVE: gear_feed = "Driving"
        elif gear == PARKING: gear_feed = "Parking"
        elif gear == REVERSE: gear_feed = "Reverse"   
        print("changing gear ...")
        delay = 0
        while self.info_1_dict["Gear_shift_Feedback"] != gear_feed:
          #  self.clock.tick_busy_loop(10)
            
            self.get_feedback()
            # print("gear:", gear, "cur gear: ", self.info_1_dict["Gear_shift_Feedback"])

            if self.info_2_dict["Vehicle_Speed"] <= 1: #and self.info_1_dict["Brake_ACT_Feedback"] >= 900:
                print("delay : ",delay)
                if delay >= 400:
                    #print("please change the gear")
                    self.driving_cmd_dict["Gear_shift_CMD"] = gear
                    self.send_control()
                    delay = 0
                delay += 1
            else:
                print("not ok to change gear")
                self.driving_cmd_dict["Brake_CMD"] = min(self.driving_cmd_dict["Brake_CMD"]+200, 15000)
                self.send_control()
                sleep(0.04)
                print("break_CMD : %d Feedback : %d Speed : %d"% (self.driving_cmd_dict["Brake_CMD"],self.info_1_dict["Brake_ACT_Feedback"], self.info_2_dict["Vehicle_Speed"]))
                delay = 0
            

    def get_vehicle_info_1(self):
        try:
            msg = self.bus.recv()  # wait 0.5 sec to get msg then raise error
            if msg.arbitration_id == self.vehicle_info_1_msg.frame_id:
                data = self.db.decode_message(msg.arbitration_id, msg.data)
                self.info_1_dict = data
        except:
            return 0

    def get_vehicle_info_2(self):
        try:
            msg = self.bus.recv()  # wait 0.5 sec to get msg then raise error
            if msg.arbitration_id == self.vehicle_info_2_msg.frame_id:
                data = self.db.decode_message(msg.arbitration_id, msg.data)
                self.info_2_dict = data

        except:
            return 0

    def send_control_cmd(self):
        # print("_send_command")
        self.control_cmd_dict['Alive_Count'] = (self.control_cmd_dict['Alive_Count'] + 1) % 256
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    

    def send_driving_cmd(self):
        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
