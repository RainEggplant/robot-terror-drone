#!/usr/bin/python3
#coding=utf8
import cv2
import sys
import numpy as np
import pid
import time
import math
import socket
import threading
import PWMServo

print('''
**********************************************************
*******功能:识别对应颜色的小球,让摄像头跟随小球转动*******
**********************************************************
----------------------------------------------------------
Official website:http://www.lobot-robot.com/pc/index/index
Online mall:https://lobot-zone.taobao.com/
----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Usage:
  -d | --显示摄像头图像，并将识别的颜色对应的物体圈出,此模
         式下后面接的识别颜色仅此次生效
  -n | --只运行程序不显示图像,此模式下识别的颜色仅此次生效
  -r | --表示要识别的颜色为red
  -g | --表示要识别的颜色为green
  -b | --表示要识别的颜色为blue
----------------------------------------------------------
Example #1:
 显示图像,圈出识别到的蓝色物体，并跟随
  python3 cv_track_stream.py -d -b
----------------------------------------------------------
Example #2:
 不显示图像,识别红色物体，并跟随
  python3 cv_track_stream.py -n -r
----------------------------------------------------------
Version: --V2.2  2019/05/24
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可中断此次程序运行
----------------------------------------------------------
''')

target_color = "green"

if len(sys.argv) == 3:
    if sys.argv[1] == '-d':
        Running = True
        debug = True
        target_color = sys.argv[2]
        if target_color == "-r":
            target_color = "red"
        elif target_color == "-g":
            target_color = "green"
        elif target_color == "-b":
            target_color = "blue"
        else:
            print("异常：颜色参数输入错误！")
            sys.exit()
    elif sys.argv[1] == '-n':
        Running = True
        debug = False
        target_color = sys.argv[2]
        if target_color == "-r":
            target_color = "red"
        elif target_color == "-g":
            target_color = "green"
        elif target_color == "-b":
            target_color = "blue"
        else:
            print("异常：颜色参数输入错误！") 
            sys.exit()
    else:
        print("异常：模式参数输入错误！")
        sys.exit()
elif len(sys.argv) == 2:
    if sys.argv[1] == '-d':
        Running = True
        debug = True
    elif sys.argv[1] == '-n':
        Running = True
        debug = False
    else:
        print("异常：模式参数输入错误！")
        sys.exit()
else:
    debug = False
    Running = False

#摄像头默认分辨率640x480,处理图像时会相应的缩小图像进行处理，这样可以加快运行速度
#缩小时保持比例4：3,且缩小后的分辨率应该是整数
c = 60
width, height = c*4, c*3
resolution = str(width) + "x" + str(height)

print('''
--程序正常运行中......
--分辨率:{0}                                              
--识别颜色:{1}                                            
'''.format(resolution, target_color))

orgFrame = None
ret = False
radius = 0
dis_ok = False

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

#数值范围映射
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#获取面积最大的轮廓
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None;

        for c in contours :
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算面积
            if contour_area_temp > contour_area_max : #新面积大于历史最大面积就将新面积设为历史最大面积
                contour_area_max = contour_area_temp
                if contour_area_temp > 100: #只有新的历史最大面积大于100,才是有效的最大面积
                                           #就是剔除过小的轮廓
                    area_max_contour = c

        return area_max_contour #返回得到的最大面积，如果没有就是 None

def get_image():
    global orgFrame
    global ret
    global Running
    global cap
    global width, height
    while True:
        if Running:
            if cap.isOpened():
                ret, orgFrame = cap.read()
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

th1 = threading.Thread(target = get_image)
th1.setDaemon(True)
th1.start()

servo_1 = 1
servo_2 = 2
PWMServo.setServo(servo_1, 1500, 500)
PWMServo.setServo(servo_2, 1500, 500)
time.sleep(0.5)

def Move():
    global servo_1
    global servo_2
    global dis_ok, x_dis, y_dis
    
    while True:
    #云台跟踪
        if dis_ok is True:
            PWMServo.setServo(servo_1,y_dis,20)
            PWMServo.setServo(servo_2,x_dis,20)
            dis_ok = False
        else:
            time.sleep(0.01)
     
#作为子线程开启
th2 = threading.Thread(target=Move)
th2.setDaemon(True)
th2.start()

x_pid = pid.PID(P=0.1, I=0.01, D=0)#pid初始化
y_pid = pid.PID(P=0.1, I=0.01, D=0)
x_dis = 1500
y_dis = 1500

#颜色的字典
color_range = {'red': [(0,43,46), (6, 255, 255)],
              'blue': [(110,43,46), (124, 255,255)],
              'green': [(35,43,46), (77, 255, 255)],
              }

range_rgb = {'red': (0, 0, 255),
              'blue': (255, 0,0),
              'green': (0, 255, 0),
              }

while True:
  if orgFrame is not None and ret:
    t1 = cv2.getTickCount()
    orgframe = cv2.resize(orgFrame, (width,height), interpolation = cv2.INTER_CUBIC) #将图片缩放到 320*240         
    img_center_x = orgframe.shape[:2][1]/2#获取缩小图像的宽度值的一半
    img_center_y = orgframe.shape[:2][0]/2
    frame = cv2.GaussianBlur(orgframe, (3,3), 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #将图像转换到HSV空间

    #分离出各个HSV通道
    h, s, v = cv2.split(frame)
    v = cv2.equalizeHist(v)
    frame = cv2.merge((h,s,v))

    frame = cv2.inRange(frame, color_range[target_color][0], color_range[target_color][1]) #根据hsv值对图片进行二值化 
    frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, (3,3))
    (_, contours, hierarchy) = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #找出所有外轮廓
    areaMaxContour = getAreaMaxContour(contours) #找到最大的轮廓
    
    centerX = 0
    centerY = 0
    radius = 0
    
    if areaMaxContour is not None:  #有找到最大面积
        (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour) #获取最小外接圆
        cv2.circle(orgframe, (int(centerX), int(centerY)), int(radius), range_rgb[target_color], 2) 
        if radius >= 3:
            ########pid处理#########
            #x2处理的是控制水平的舵机，y2处理控制竖直的舵机#
            #以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            x_pid.SetPoint = img_center_x#设定
            x_pid.update(centerX)#当前
            x_pwm = x_pid.output#输出
            x_dis += x_pwm
            x_dis = int(x_dis)
            if x_dis < 500:
                x_dis = 500
            elif x_dis > 2500:
                x_dis = 2500
            y_pid.SetPoint = img_center_y
            y_pid.update(centerY)
            y_pwm = y_pid.output
            y_dis -= y_pwm
            y_dis = int(y_dis)
            if y_dis < 600:
                y_dis = 600
            elif y_dis > 2000:
                y_dis = 2000
            dis_ok = True
                                   
    t2 = cv2.getTickCount()
    time_r = (t2 - t1) / cv2.getTickFrequency()               
    fps = 1.0/time_r
    if debug:
        cv2.putText(orgframe, "fps:" + str(int(fps)),
                (10, orgframe.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
        cv2.imshow("orgframe", orgframe)
        cv2.waitKey(1)
  else:
    time.sleep(0.01)     
cap.release()
cv2.destroyAllWindows()
