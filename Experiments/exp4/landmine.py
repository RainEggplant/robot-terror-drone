# !/usr/bin/python3
# coding=utf8

import time
import numpy as np
import sys
import cv2
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
H_THRESHOLD = 130
FALL_THRESHOLD = 20
RUNNING = 1
leftdownpoint=(W/6,2/3*H)
rightdownpoint=(W/6*5,2/3*H)
frontpoint=(W/2,H/2)
PI=3.1415926535
XTHRESOLDS=W/2
THETA_VER_THRESHOLD=8
THETA_HORI_THRESHOLD=8

rightflag = 1
somersault = 0
check=0
leftdata=[]
rightdata=[]
#go to rightS
state=2
#0-go straight and stop
#1-step
#2-landmine
#3-gap
#4-door
# get theta

def linefit(a):
    a=np.array(a)
    len_a=np.size(a,0)
    ones=np.ones(len_a).reshape(-1,1)
    A=np.hstack((a,ones))
    B=np.dot(A.T,A)
    D,V=np.linalg.eig(B)
    D=np.abs(D)
    vec=V[:,np.where(D==np.min(D))]
    vec.reshape(-1,1)
    k=-vec[0]/vec[1]
    b=-vec[2]/vec[1]
    return k,b

def gethoritheta(ditch):
    ditch=ditch.reshape(-1,2)
    quap=ditch[ditch.argsort(axis=0)[:,0]]
    k1=(quap[0,1]-quap[2,1])/(quap[0,0]-quap[2,0])
    k2=(quap[1,1]-quap[3,1])/(quap[1,0]-quap[3,0])
    k=(k1+k2)/2
    theta=np.arctan(k)*180/PI
    return theta
  
def getvertheta(track,left):
    track=track.reshape(-1,2)
    if left==1:
        indm=np.where((track[:,0]<XTHRESOLDS)==1)
    else:
        indm=np.where((track[:,0]>XTHRESOLDS)==1)
    track=track[indm]
    k,b=linefit(track)
    k = 1/k
    theta=np.arctan(k)*180/PI
    return theta

# plan to act
def plan2act(plan_act):
    if plan_act is not None:
        for i in plan_act:
            SSR.running_action_group(i, 1)
            print('\t action= ',i)
def setcamera(state):
    global check
    if check==0:
        if state==4:
            PWMServo.setServo(1, 1700, 500)
            #time.sleep(0.6)
            PWMServo.setServo(2, 1500, 500)
            #time.sleep(0.6)
        else:
            PWMServo.setServo(1, 2100, 500)
            #time.sleep(0.6)
            PWMServo.setServo(2, 1500, 500)
            #time.sleep(0.6)
    elif check==1:
        print('check',check)
        PWMServo.setServo(1, 2100, 500)
        PWMServo.setServo(2, 1200, 500)
        check=2
    elif check==2:
        print('check',check)
        PWMServo.setServo(1, 2100, 500)
        PWMServo.setServo(2, 1700, 500)
        check=3
    else:
        print('check',check)
        PWMServo.setServo(1, 2100, 500)
        #time.sleep(0.6)
        PWMServo.setServo(2, 1500, 500)
        #time.sleep(0.6)
        check=0

def setstate(state,data):
    frontbrink=0
    yellow=0
    red=0
    theta=0
    for i in data['light']:
        if i == 'red':
            print('find red')
            red=1
        if i=='yellow':
            print('find yellow')
            yellow=1
    if 'track' in data:
        approx = data['track']
        if (cv2.pointPolygonTest(approx,frontpoint,0)==-1):
            frontbrink=1
    else:
        print('!!!!!! not find the track, state change warning!!!!!!')
    if (state==0)&(red==1):
        state=1
        return state
    if (state==1):
        state=2
        return state
    if (state==2)&(frontbrink):
        state=3
        return state
    if (state==3) and (somersault):
        state=4
        return state
    return state
def plan0():
    plan_act=[]
    plan_act.append('custom/walk')
    return plan_act
def plan1():
    plan_act=[]
    plan_act.append('custom/walk')
    plan_act.append('custom/walk')
    plan_act.append('custom/somersault_step')
    plan_act.append('custom/somersault_step')
    plan_act.append('custom/turn_to_left')
    plan_act.append('custom/turn_to_left')
    plan_act.append('custom/turn_to_left')
    plan_act.append('custom/turn_to_left')
    plan_act.append('custom/move_left_bit')
    plan_act.append('custom/move_left_bit')
    return plan_act
def plan2(data):
    global rightflag
    plan_act = []
    rightbrink=0
    leftbrink=0
    line_judge = 0
    approx = data['track']
    if (cv2.pointPolygonTest(approx,leftdownpoint,0)==-1):
        leftbrink=1
    if (cv2.pointPolygonTest(approx,rightdownpoint,0)==-1):
        rightbrink=1
    # brink judge
    if leftbrink:
        rightflag = 1
        print('find the left line')
        plan_act.append('custom/move_right_bit')
        line_judge = 1
    if rightbrink:
        rightflag = 0
        print('find the left line')
        plan_act.append('custom/move_left_bit')
        line_judge = 1
    # landmine judge
    if (len(data['landmine']) > 0):
        maxh = max(np.array(data['landmine']).reshape(2, -1)[1, :])
        if maxh < H_THRESHOLD:
            plan_act.append('custom/walk_12_11')
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
    print('rightflag:  ',rightflag)
    return plan_act
def plan3(data):
    global somersault
    global check
    plan_act=[]
    if 'ditch' not in data:
        print('!!!!!!not find ditch warning!!!!!!')
        print('change the check')
        check = 1
        return plan_act
    theta=gethoritheta(data['ditch'])
    if np.abs(theta)<THETA_HORI_THRESHOLD:
        plan_act.append('custom/walk')
        plan_act.append('custom/145')
        somersault = 1
    elif theta<0:
        print('need to turn to right')
        plan_act.append('custom/turn_to_right')
        plan_act.append('custom/walk')
        plan_act.append('custom/145')
        somersault = 1
    else:
        print('need to turn to left')
        plan_act.append('custom/turn_to_left')
        plan_act.append('custom/walk')
        plan_act.append('custom/145')
        somersault = 1
    return plan_act
  
def plan4(data):
    plan_act=[]
    yellow=0
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
    plan_act.append('custom/pa')
            
    #if not data['block']:
    #    print('block:  0')
    #    plan_act.append('custom/walk')
    #else:
    #    print('block:  1')
    return plan_act
  
def plancheck(data):
    plan_act=[]
    global leftdata
    global rightdata
    if 'ditch' not in data:
        if 'ditch' in leftdata:
            plan_act.append('custom/turn_to_left')
            return plan_act
        if 'ditch' in rightdata:
            plan_act.append('custom/turn_to_right')
            return plan_act
        else:
            print('!!!!!!!warning not find the ditch in left and right and front data!!!')
            plan_act.append('custom/walk')
    return plan_act
  
def plancheck(data):
    plan_act=[]
    global leftdata
    global rightdata
    print('go to the check stageS')
    if 'ditch' not in data:
        if 'ditch' in leftdata:
            plan_act.append('custom/turn_to_left')
            return plan_act
        if 'ditch' in rightdata:
            plan_act.append('custom/turn_to_right')
            return plan_act
        else:
            print('!!!!!!!warning not find the ditch in left and right and front data!!!')
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
    plan_act=[]
    #setcamera
    setcamera(state)
    #getdata
    data = img_proc.analyze_objects()
    print('\n main process','\t state=',state)
    if ('track' not in data)and ('ditch' not in data)and('track_next' not in data):
        print('!!!!!!recognize nothing')
        continue
    #
    if check!=0:
        print('go to the check interrupt stage\ncheck:',check)
        if check==1:
            leftdata=data
        if check==2:
            rightdata=data
        if check==3:
            plan_act=plancheck(data)
    else:
        #get and change the state main stage
        state=setstate(state,data)
        plan_act = []
        if state==0:
            plan_act=plan0()
        if state==1:
            plan_act=plan1()
        if state==2:
            plan_act=plan2(data)
        if state==3:
            plan_act=plan3(data)
        if state==4:
            plan_act=plan4(data)
    plan2act(plan_act)
    time.sleep(0.5)
print('Press Enter to exit.')
input()
