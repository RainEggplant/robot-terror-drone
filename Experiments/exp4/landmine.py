#!/usr/bin/python3
# coding=utf8
import threading
import math
import time
import numpy as np
import sys
import cv2

sys.path.append('../..')  # nopep8
import PWMServo
import Serial_Servo_Running as SSR


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
        Running = True
        debug = True
    elif sys.argv[1] == '-n':
        Running = True
        debug = False
    else:
        print("异常：参数输入错误！")
        sys.exit()
else:
    debug = False
    Running = False

# 摄像头默认分辨率640x480,处理图像时会相应的缩小图像进行处理，这样可以加快运行速度
# 缩小时保持比例4：3,且缩小后的分辨率应该是整数
c = 80
width, height = c*4, c*3
resolution = str(width) + "x" + str(height)

print('''
--程序正常运行中......
--分辨率:{0}                                              
'''.format(resolution))


color = 0
rR = 0
rG = 0
rB = 0
rBL = 0
rW = 0
radius = 0

orgFrame = None
ret = False
Color_BGR = (0, 0, 0)

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

PWMServo.setServo(1, 2000, 500)
PWMServo.setServo(2, 1500, 500)
time.sleep(0.5)


# 数值映射
# 将一个数从一个范围映射到另一个范围
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓


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


th1 = threading.Thread(target=get_image)
th1.setDaemon(True)
th1.start()

get_color = False
color_max = None


def runAction():
    global get_color
    global color_max
    is_walking = False
    while True:
        if get_color:
            if color_max == 'red':
                is_walking = False
                get_color = False
            elif color_max == 'green':
                is_walking = True
                # SSR.running_action_group('custom/walk', 1)
                get_color = False
            elif color_max == 'black':
                is_walking = False
                # SSR.running_action_group('custom/walk', 1)
                get_color = False
            else:
                get_color = False
                if is_walking:
                    pass
                    # SSR.running_action_group('custom/walk', 1)
        else:
            if is_walking:
                pass
                # SSR.running_action_group('custom/walk', 1)

        time.sleep(0.01)


# 启动动作运行子线程
th2 = threading.Thread(target=runAction)
th2.setDaemon(True)
th2.start()

# 颜色的字典
color_range = {'red': [(0, 43, 46), (6, 255, 255)],
               'green': [(54, 43, 46), (77, 255, 255)],
               'yellow': [(30, 43, 46), (50, 255, 255)],
               'black': [(0, 0, 0), (255, 255, 5)]
               }

range_rgb = {'red': (0, 0, 255),
             'green': (0, 255, 0),
             'yellow': (255, 255, 0),
             'black': (0, 0, 0)
             }

while True:
    if orgFrame is not None and ret:
        t1 = cv2.getTickCount()
        orgframe = cv2.resize(orgFrame, (width, height),
                              interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(orgframe, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        # 分离出各个HSV通道
        h, s, v = cv2.split(frame)
        # 直方图化
        v = cv2.equalizeHist(v)
        # 合并三个通道
        Frame = cv2.merge((h, s, v))

        color_list = []
        rad = 0
        areaMaxContour = 0
        max_area = 0
        area_max = 0
        centerX = 0
        centerY = 0

        for i in color_range:
            frame = cv2.inRange(
                Frame, color_range[i][0], color_range[i][1])  # 对原图像和掩模进行位运算
            opened = cv2.morphologyEx(
                frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            closed = cv2.morphologyEx(
                opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
            (image, contours, hierarchy) = cv2.findContours(
                closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

            if areaMaxContour is not None:
                if area_max > max_area:  # 找最大面积
                    max_area = area_max
                    color_max = i
                    areaMaxContour_max = areaMaxContour

        if max_area != 0:
            ((centerX, centerY), rad) = cv2.minEnclosingCircle(
                areaMaxContour_max)  # 获取最小外接圆
            centerX, centerY, rad = int(centerX), int(
                centerY), int(rad)  # 获取圆心，半径
            cv2.circle(orgframe, (centerX, centerY),
                       rad, (0, 255, 0), 2)  # 画出圆心

            if color_max == 'red':  # 红色最大
                # print("red")
                Color_BGR = range_rgb["red"]
                if get_color is False:
                    get_color = True
            elif color_max == 'green':  # 绿色最大
                Color_BGR = range_rgb["green"]
                # print("green")
                if get_color is False:
                    get_color = True
            elif color_max == 'yellow':  # 黄色最大
                Color_BGR = range_rgb["yellow"]
                # print("yellow")
                if get_color is False:
                    get_color = True
            elif color_max == 'black':  # 黑色最大
                Color_BGR = range_rgb["black"]
                # print("black")
                if get_color is False:
                    get_color = True
            else:
                Color_BGR = (0, 0, 0)
                color_max = "Other"
        else:
            Color_BGR = (0, 0, 0)
            color_max = "None"

        t2 = cv2.getTickCount()
        time_r = (t2 - t1) / cv2.getTickFrequency()
        fps = 1.0/time_r
        if debug:
            cv2.putText(orgframe, "Color: " + color_max, (10,
                                                          orgframe.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, Color_BGR, 2)
            cv2.putText(orgframe, "fps:" + str(int(fps)),
                        (10, orgframe.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)  # (0, 0, 255)BGR
            cv2.imshow("orgframe", orgframe)
            cv2.waitKey(1)
    else:
        time.sleep(0.01)
cap.release()
cv2.destroyAllWindows()
