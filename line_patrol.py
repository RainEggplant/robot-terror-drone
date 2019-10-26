#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# 小球颜色识别后执行动作组

import cv2
import numpy as np
import time
import threading
import math
import Serial_Servo_Running as SSR
import signal
import PWMServo

debug = 1
Running = False
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

orgFrame = None
get_image_ok = False
get_line = False

go_straight = '1'
turn_left   = 'turn_left'
turn_right  = 'turn_right'

ori_width  =  int(4*160)#原始图像640x480
ori_height =  int(3*160)
resize_width   = int(4*20)#处理图像时缩小为80x60,加快处理速度，谨慎修改！
resize_height  = int(3*20)
line_color     = (255, 0, 0)#图像显示时，画出的线框颜色
line_thickness = 2         #图像显示时，画出的线框的粗细
# 机器人应该转的角度
deflection_angle = 0

PWMServo.setServo(1, 2000, 500)
PWMServo.setServo(2, 1500, 500)
time.sleep(0.5)
#SSR.running_action_group('turn_left', 1000)
# 暂停信号的回调
def cv_stop(signum, frame):
    global Running
    global ncount
    global stop
    global count
    global state

    
    print("物体跟踪_停止")    
    if Running is True:
        ncount = 0
        state  = False
        stop   = False
        count  = 0

        Running = False
        
# 继续信号的回调
def cv_continue(signum, frame):
    global Running
    global stop_run
    
    print("物体跟踪_开始")
    if Running is False:
        stop_run = False
        SSR.running_action_group('0', 1)
        Running = True

#   注册信号回调
signal.signal(signal.SIGTSTP, cv_stop)
signal.signal(signal.SIGCONT, cv_continue)

class Point(object):
    x = 0
    y = 0

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Line(object):
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

def GetCrossAngle(l1, l2):
    '''
    求两直线之间的夹角
    :param l1:
    :param l2:
    :return:
    '''
    arr_0 = np.array([(l1.p2.x - l1.p1.x), (l1.p2.y - l1.p1.y)])
    arr_1 = np.array([(l2.p2.x - l2.p1.x), (l2.p2.y - l2.p1.y)])
    cos_value = (float(arr_0.dot(arr_1)) / (np.sqrt(arr_0.dot(arr_0)) * np.sqrt(arr_1.dot(arr_1))))   # 注意转成浮点数运算
    return np.arccos(cos_value) * (180/np.pi)

#映射函数
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#画圈，参数（要显示文字的图像，x坐标，y坐标，半径， 处理图像的宽度， 处理图像的高度， 颜色（可不填）， 大小（可不填））
def picture_circle(orgimage, x, y, r, resize_w, resize_h, l_c = line_color, l_t = line_thickness):
    global ori_width
    global ori_height
    
    x = int(leMap(x, 0, resize_w,  0, ori_width))
    y = int(leMap(y, 0, resize_h,  0, ori_height))
    r = int(leMap(r, 0, resize_w,  0, ori_width))   
    cv2.circle(orgimage, (x, y), r, l_c, l_t)

#检测并返回最大面积
def getAreaMaxContour(contours,area=1):
        contour_area_max = 0
        area_max_contour = None

        for c in contours :
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max : 
                contour_area_max = contour_area_temp
                if contour_area_temp > area:#面积大于1
                    area_max_contour = c
        return area_max_contour

ret = False
def get_image():
    global orgFrame
    global ret
    global Running
    global cap
    while True:
        if Running:
            if cap.isOpened():
                ret, orgFrame = cap.read()
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

# 显示图像线程
th1 = threading.Thread(target=get_image)
th1.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()

roi = [ # [ROI, weight]
        (0,  40,  0, 160, 0.5), 
        (40, 80,  0, 160, 0.3), 
        (80, 120,  0, 160, 0.2)
       ]
state = False
stop = False
count = 0
angle = 0
def Tracing(orgimage, r_w = resize_width, r_h = resize_height, r = roi, l_c = line_color, l_t = line_thickness):
    global ori_width, ori_height
    global img_center_x, img_center_y
    global deflection_angle, angle
    global get_line,state,stop,count
    #图像缩小，加快处理速度
    orgframe = cv2.resize(orgimage, (r_w, r_h), interpolation = cv2.INTER_LINEAR)
    orgframe = cv2.cvtColor(orgframe, cv2.COLOR_BGR2GRAY)#转化为灰度图像
    orgframe = cv2.GaussianBlur(orgframe, (3,3), 0)#高斯模糊，去噪
    _, Imask = cv2.threshold(orgframe, 50, 255, cv2.THRESH_BINARY_INV)#二值化
    Imask = cv2.erode(Imask, None, iterations=2)
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
    centroid_x_sum = 0
    area_sum = 0
    n = 0
    weight_sum = 0
    center_ = []
    max_area = 0
    _, cnts , _ = cv2.findContours(Imask , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)#找出所有轮廓
    cnt_large  = getAreaMaxContour(cnts, area = 1200)#找到最大面积的轮廓
    if cnt_large is not None:
        rect = cv2.minAreaRect(cnt_large)#最小外接矩形
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        box[1, 0] = int(leMap(box[1, 0], 0, r_w, 0, ori_width))
        box[1, 1] = int(leMap(box[1, 1], 0, r_h, 0, ori_height))
        box[3, 0] = int(leMap(box[3, 0], 0, r_w, 0, ori_width))
        box[3, 1] = int(leMap(box[3, 1], 0, r_h, 0, ori_height))
        box[0, 0] = int(leMap(box[0, 0], 0, r_w, 0, ori_width))
        box[0, 1] = int(leMap(box[0, 1], 0, r_h, 0, ori_height))
        box[2, 0] = int(leMap(box[2, 0], 0, r_w, 0, ori_width))
        box[2, 1] = int(leMap(box[2, 1], 0, r_h, 0, ori_height))
        cv2.drawContours(orgimage, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形
        cv2.line(orgimage, (box[1, 0],box[1, 1]), (box[3, 0],box[3, 1]), l_c, l_t)
        cv2.line(orgimage, (box[0, 0],box[0, 1]), (box[2, 0],box[2, 1]), l_c, l_t)    
        angle = int(GetCrossAngle(Line(Point(box[1, 0],box[1, 1]), Point(box[3, 0],box[3, 1])), Line(Point(box[0, 0],box[0, 1]), Point(box[2, 0],box[2, 1]))))
        if 60 < angle < 120:
            count +=1
            if count >= 1:
                count = 0
                state = True
        if state:
            if 60 < angle < 120:
                stop = True
    for r in roi:
        n += 1
        blobs = Imask[r[0]:r[1], r[2]:r[3]]
        _, cnts , _ = cv2.findContours(blobs , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)#找出所有轮廓
        cnt_large  = getAreaMaxContour(cnts)#找到最大面积的轮廓
        if cnt_large is not None:
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            box[0, 1], box[1, 1], box[2, 1], box[3, 1] = box[0, 1] + (n - 1)*r_w/4, box[1, 1] + (n - 1)*r_w/4, box[2, 1] + (n - 1)*r_w/4, box[3, 1] + (n - 1)*r_w/4
            box[1, 0] = int(leMap(box[1, 0], 0, r_w, 0, ori_width))
            box[1, 1] = int(leMap(box[1, 1], 0, r_h, 0, ori_height))
            box[3, 0] = int(leMap(box[3, 0], 0, r_w, 0, ori_width))
            box[3, 1] = int(leMap(box[3, 1], 0, r_h, 0, ori_height))
            box[0, 0] = int(leMap(box[0, 0], 0, r_w, 0, ori_width))
            box[0, 1] = int(leMap(box[0, 1], 0, r_h, 0, ori_height))
            box[2, 0] = int(leMap(box[2, 0], 0, r_w, 0, ori_width))
            box[2, 1] = int(leMap(box[2, 1], 0, r_h, 0, ori_height))
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            area = cv2.contourArea(box)
            cv2.drawContours(orgimage, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形            
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点
            center_.append([center_x,center_y])            
            cv2.circle(orgimage, (int(center_x), int(center_y)), 10, (0,0,255), -1)#画出中心点
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum is not 0:
        center_x_pos = centroid_x_sum / weight_sum
        #中间公式
        deflection_angle = 0.0
        deflection_angle = -math.atan((center_x_pos - img_center_x/2)/(img_center_y/2))
        deflection_angle = deflection_angle*180.0/math.pi
        #print(center_x_pos)
         #框中心画十字
        #cv2.line(orgimage, (img_center_x/2, img_center_y), (int(center_x_pos), img_center_y/2), l_c, l_t)         
    get_line = True
    
stop_run = False        
ncount = 0
state1 = 0
last_state = 0
def move():
    global deflection_angle, angle
    global get_line
    global go_straight,turn_left,turn_right
    global ncount,stop,stop_run,state1,last_state
    while True:
        if stop_run is False:
            if stop is False:
                if get_line:
                    get_line = False
                    if -25 <= deflection_angle <= 25:
                        state1 = 1
                        SSR.running_action_group(go_straight, 1)           
                    elif deflection_angle > 25:
                        state1 = 2
                        SSR.running_action_group(turn_left, 1)
                    elif deflection_angle < -25:
                        state1 = 3
                        SSR.running_action_group(turn_right, 1)
                else:
                    time.sleep(0.01)
            else:
                ncount += 1
                if ncount > 6:
                    stop_run = True
                if get_line:
                    get_line = False
                    if -25 <= deflection_angle <= 25:
                        state1 = 1
                        SSR.running_action_group(go_straight, 1)           
                    elif deflection_angle > 25:
                        state1 = 2
                        SSR.running_action_group(turn_left, 1)
                    elif deflection_angle < -25:
                        state1 = 3
                        SSR.running_action_group(turn_right, 1)
                else:
                    time.sleep(0.01)
            last_state = state1
            
        else:
            time.sleep(0.01)
      
th2 = threading.Thread(target=move)
th2.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th2.start()    

SSR.running_action_group('0', 1)
while True:
    if orgFrame is not None and ret:
        if Running:
            t1 = cv2.getTickCount()
            img_center_x = orgFrame.shape[:2][1]
            img_center_y = orgFrame.shape[:2][0]
            Tracing(orgFrame, 160, 120)    
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0/time_r
            if debug == 1:#调试模式下
                cv2.putText(orgFrame, "state stop:" + str(state) + str(stop) ,
                        (10, orgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(orgFrame, "fps:" + str(int(fps)),
                        (10, orgFrame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
                cv2.namedWindow('orgFrame', cv2.WINDOW_AUTOSIZE)#显示框命名
                cv2.moveWindow('orgFrame', img_center_x, 100)#显示框位置
                cv2.imshow('orgFrame', orgFrame) #显示图像
                cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()
            time.sleep(0.01)
    else:
        time.sleep(0.01)
cv2.destroyAllWindows()
