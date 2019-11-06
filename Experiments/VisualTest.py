#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# Place this file on /home/pi/human_code and run

import cv2
import numpy as np
import time
import threading
import math
import sys

sys.path.append('..')  # nopep8
import Serial_Servo_Running as SSR
import signal
import PWMServo

debug = 1
Running = True
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
#cap = cv2.VideoCapture(stream)
cap = cv2.VideoCapture(stream)

orgFrame = None

# 机器人应该转的角度
deflection_angle = 0

# The value must be modified according to how the camera base was placed
PWMServo.setServo(1, 1500, 500)
PWMServo.setServo(2, 1500, 500)  # Same as above

#SSR.running_action_group('1', 5)

if Running:
    if cap.isOpened():
        ret, orgFrame = cap.read()
    else:
        time.sleep(0.01)
        print('cap not open\n')

if orgFrame is not None and ret:
    np.save('captured', orgFrame)
    cv2.imshow('orgFrame', orgFrame)  # 显示图像
    cv2.waitKey(1)
else:
    print('orgFrame read error\n')
print(ret)
