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

from can_module import CAN
from pygame import time

NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7
FPS = 50

def main():
    can = CAN()
    #can.send_control()
    while True:
        can.clock.tick_busy_loop(FPS)
        can.send_control_cmd() #alive count 
        can.get_feedback()
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("End of Alive Count ")
