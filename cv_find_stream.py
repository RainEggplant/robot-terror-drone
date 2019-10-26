#!/usr/bin/python3
#coding=utf8
import cv2
import time
import math
import sys
import socket
import threading
import PWMServo

print('''
**********************************************************
*功能：通过摄像头检测特定颜色端物体，使机器人跟随物体运动*
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
         式下识别的颜色仅此次生效
  -n | --只运行程序不显示图像,此模式下识别的颜色仅此次生效
  -r | --表示要识别的颜色为red
  -g | --表示要识别的颜色为green
  -b | --表示要识别的颜色为blue
----------------------------------------------------------
Example #1:
 显示图像,圈出识别到的蓝色物体，并跟随
  python3 cv_find_stream.py -d -b
----------------------------------------------------------
Example #2:
 不显示图像,识别红色物体，并跟随
  python3 cv_find_stream.py -n -r
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
c = 80
width, height = c*4, c*3
resolution = str(width) + "x" + str(height)

print('''
--程序正常运行中......
--分辨率:{0}                                              
--识别颜色:{1}                                            
'''.format(resolution, target_color))

step = 0
Dist = 0.0
rads = [0,0,0,0,0,0,0,0,0,0]
lastR = 0
count = 0
centerX = 0
centerY = 0 
xc = False

orgFrame = None
ret = False
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

#数值映射
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#找出最大的轮廓
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None;

        for c in contours : #历便所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算面积
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > 50:  #限制最小面积
                    area_max_contour = c

        return area_max_contour  #返回最大面积

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

def logic():
    global step
    global Dist
    global lsc
    global xc
    global centerX
    global centerY
    global Running
    
    while True:
        if Running is True:
            if xc is True:
                if step == 0:
                    if centerX > 440:  #不在中心，根据方向让机器人转向一步
                        lsc.RunActionGroup(4,1)
                        lsc.WaitForFinish(3000)
                        time.sleep(0.3)
                    elif centerX < 200:
                        lsc.RunActionGroup(3,1)
                        lsc.WaitForFinish(3000)
                        time.sleep(0.3)
                    else:
                        step = 1 #转到步骤1
                        pass
                elif step == 1:
                    if Dist > 30:  #检查距离是不是在 30 到 20之间，根据情况做对应操作
                        lsc.RunActionGroup(1,1)  #大于30 就前进，去靠近目标
                        lsc.WaitForFinish(3000) #等待动作执行完
                    elif Dist < 20 and Dist > 0:  
                        lsc.RunActionGroup(2,1)#小于20就后退，去远离目标
                        lsc.WaitForFinish(3000) #等待动作执行完
                    else:
                        pass
                    step = 0 #回到步骤0
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)
            xc = False
        else:
            time.sleep(0.01)

#启动跟随控制六足动作的线程
th2 = threading.Thread(target=logic)
th2.setDaemon(True)
th2.start()

#颜色的字典
color_range = {'red': [(0,43,46), (6, 255, 255)],
              'blue': [(110,43,46), (124, 255,255)],
              'green': [(35,43,46), (77, 255, 255)],
              }

range_rgb = {'red': (0, 0, 255),
              'blue': (255, 0,0),
              'green': (0, 255, 0),
              }

pitch = 1500
yaw = 1500
lsc.MoveServo(19, 1500,1000)  #先让摄像头云台舵机转到对于中间位置
lsc.MoveServo(20, 1500,1000)

while True:
  if orgFrame is not None and ret:
    t1 = cv2.getTickCount()
    orgframe = cv2.resize(orgFrame,(width, height), interpolation = cv2.INTER_CUBIC)
    frame = cv2.GaussianBlur(orgframe, (3,3), 0) #高斯模糊
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #将图片转换到HSV空间

    #分离出各个HSV通道
    h, s, v = cv2.split(frame)
    v = cv2.equalizeHist(v)
    frame = cv2.merge((h,s,v))

    frame = cv2.inRange(frame, color_range[target_color][0], color_range[target_color][1])  #根据目标的颜色对图片进行二值化
    frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, (4,4)) #闭操作
    (_, contours, hierarchy) = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #找到所有的轮廓
    areaMaxContour = getAreaMaxContour(contours)  #找到其中最大的轮廓
    centerX = 0
    centerY = 0 

    if areaMaxContour is not None:
        ((centerX, centerY), rad) = cv2.minEnclosingCircle(areaMaxContour) #获得最小外接圆
        cv2.circle(orgframe,(int(centerX), int(centerY)), int(rad), range_rgb[target_color], 2)
        centerX = leMap(centerX, 0.0, width, 0.0, 640.0)  #将数据从0-width 映射到 0-640
        centerY = leMap(centerY, 0.0, height, 0.0, 480.0)
        rads.append(rad) #追加新的最小外接圆半径
    else:
        rads.append(0)  #没找到球就追加0
    rads = rads[1:]  #将最久的一个数据去掉
    
    ######根据图像计算距离##########
    alr = 0
    for r in rads:
        alr = alr + r
    alr = alr / 10  #计算最近的10次的最小外接圆直径
    lastR = lastR * 0.7 + alr * 0.3  #简单滤波
    count += 1
    if count >= 5:  #每5次更新一次距离
        if lastR >= 2:  #半径太小就直接让就距离为0, 0就是没有目标
            Dist = 40 * 47.0 / (lastR * 2) # 40 是我们的目标小球的，47.0是我们我们的摄像头焦距 
            Dist = Dist if Dist < 100 else 0 #限制量程
        else:
            Dist = 0
        count = 0
    ####################################
        
    if areaMaxContour is not None:
        yc = True
        if centerX > 0:
            xc = True
        else:
            xc = False

        if centerY > 450:
            yaw = yaw + 40
        elif centerY > 380:
            yaw = yaw + 25
        elif centerY > 310:
            yaw = yaw + 15
        elif centerY > 245:
            yaw = yaw + 2
        elif centerY > 235:
            yc = False
        elif centerY > 170:
            yaw = yaw - 2
        elif centerY > 100:
            yaw = yaw - 15
        elif centerY > 30:
            yaw = yaw - 25
        elif centerY > 0:
            yaw = yaw - 40
        else:
            yc = False
    else:
        pass
    t2 = cv2.getTickCount()
    time_r = (t2 - t1) / cv2.getTickFrequency()
    fps = 1.0/time_r
    if debug:
        cv2.putText(orgframe, "Distance:" + str("%.1f" %Dist),
                (10, orgframe.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        cv2.putText(orgframe, "fps:" + str(int(fps)),
                (10, orgframe.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
        cv2.imshow("orgFframe", orgframe)
        cv2.waitKey(1)
    get_image_ok = False
  else:
     time.sleep(0.01)      
cap.release()
cv2.destroyAllWindows()
