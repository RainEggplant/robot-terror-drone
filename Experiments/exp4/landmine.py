# !/usr/bin/python3
# coding=utf8

import threading
import math
import time
import numpy as np
import sys
import cv2
from image_processor import ImageProcessor
from action_planning import ActionPlanning

sys.path.append('../..')  # nopep8
import Serial_Servo_Running as SSR
import PWMServo

print('''
**********************************************************
**************功能: 响应红绿灯，并通过地雷阵。**************
**********************************************************
----------------------------------------------------------
Usage:
  -d | --显示摄像头图像，并将识别的颜色对应的物体圈出
  -n | --不显示摄像头图像
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可中断此次程序运行
----------------------------------------------------------
''')

if len(sys.argv) == 2:
    if sys.argv[1] == '-d':
        DEBUG = True
    elif sys.argv[1] == '-n':
        DEBUG = False
    else:
        print("异常：参数输入错误！")
        sys.exit()
else:
    print("异常：参数输入错误！")
    sys.exit()

rightflag=1

PWMServo.setServo(1, 2150, 500)
PWMServo.setServo(2, 1500, 500)
time.sleep(0.5)

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
img_proc = ImageProcessor(stream, DEBUG)

while 1:
    data = img_proc.get_objects_info()
    print('main process\n')
    act_plan = ActionPlanning(data)
    rightflag=act_plan.plan_action(rightflag)
    time.sleep(0.4)
    print('rightflag',rightflag)
print(data)
print('Press Enter to exit.')
input()
