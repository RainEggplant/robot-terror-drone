# !/usr/bin/python3
# coding=utf8

import time
import numpy as np
import sys
from image_processor import ImageProcessor

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

## below


W = 320
H = 240
H_THRESHOLD = 100
FALL_THRESHOLD = 20
RUNNING = 1

rightflag = 1
#go to right
state=2
#0-go straight and stop
#1-step
#2-landmine
#3-gap
#4-door
# plan to act
def plan2act(plan_act):
    for i in plan_act:
        SSR.running_action_group(i, 1)
        print('act= ',i)
def setcamera(state):
    if state==4:
        PWMServo.setServo(1, 1500, 500)
        PWMServo.setServo(2, 1500, 500)
    else:
        PWMServo.setServo(1, 2150, 500)
        PWMServo.setServo(2, 1500, 500)
def setstate(state,data):
    frontbrink=0
    yellow=0
    red=0
    for i in data['light']:
        if i == 'red':
            print('find red')
            red=1
        if i=='yellow':
            print('find yellow')
            yellow=1
    data['track'].reshape(2,-1)
    for i in data['track']:
        if (i[1]<H/3)&(i[1]>H/5):
            frontbrink=1
    if (state==0)&(red==1):
        state=1
        return state
    if (state==1):
        state=2
        return state
    if (state==2)&(frontbrink==1):
        state=3
        return state
    if (state==3):
        state=4
        return state
    return state
def plan0():
    plan_act=[]
    plan_act.append('custom/walk')
    return plan_act
def plan1():
    plan_act=[]
    plan_act.append('custom/somersault')
    return plan_act
def plan2(data):
    plan_act = []
    rightbrink=0
    leftbrink=0
    line_judge = 0
    data['track'].reshape(-1,2)
    for i in data['track']:
        if (i[1]>H/3)&(i[0]>W/5)&(i[0]<W/2):
            leftbrink=1
        if (i[1]>H/3)&(i[0]<4*W/5)&(i[0]>W/2):
            rightbrink=1
    # brink judge
    if leftbrink:
        rightflag = 1
        print('find the left line')
        time.sleep(1)
        plan_act.append('custom/move_right')
        line_judge = 1
    if rightbrink:
        rightflag = 0
        print('find the right line')
        time.sleep(1)
        plan_act.append('custom/move_left')
        line_judge = 1
    # landmine judge
    if (len(data['landmine']) > 0):
        maxh = max(np.array(data['landmine']).reshape(2, -1)[1, :])
        if maxh < H_THRESHOLD:
            plan_act.append('custom/walk')
        elif rightflag:
            print('find the landmine')
            time.sleep(1)
            plan_act.append('custom/move_right')
        else:
            print('find the landmine')
            time.sleep(1)
            plan_act.append('custom/move_left')
    elif line_judge == 0:
        plan_act.append('custom/walk')
    return plan_act
def plan3():
    plan_act=[]
    plan_act.append('custom/somersault')
    return plan_act
def plan4(data):
    plan_act=[]
    yellow=0
    for i in data['light']:
        if i=='yellow':
            print('find yellow')
            yellow=1
    if yellow==1:
        plan_act.append('custom/walk')
        plan_act.append('custom/walk')
        plan_act.append('custom/walk')
    return plan_act



#initially set head
PWMServo.setServo(1, 2150, 500)
PWMServo.setServo(2, 1500, 500)
time.sleep(0.5)
#set video
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
img_proc = ImageProcessor(stream, DEBUG)

while 1:
    #setcamera
    setcamera(state)
    #getdata
    data = img_proc.analyze_objects()
    print('\n main process','\t state=',state)
    if (len(data['track']) == 0):
        print('!!!not recognize the track')
        continue
    #get state
    state=setstate(state,data)
    plan_act = []
    if state==0:
        plan_act=plan0()
    if state==1:
        plan_act=plan1()
    if state==2:
        plan_act=plan2(data)
    if state==3:
        plan_act=plan3()
    if state==4:
        plan_act=plan4(data)
    plan2act(plan_act)
    time.sleep(0.4)
    print('rightflag', rightflag)
print(data)
print('Press Enter to exit.')
input()
