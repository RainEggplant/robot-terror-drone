# -*- coding: utf-8 -*-
import math
import numpy as np
import cv2
import threading
import time
# from matplotlib import pyplot as plt

# %% 常数定义
IMG_WIDTH = 320
IMG_HEIGHT = 240
FRONT_THRESHOLD = 200
LEFT_THRESHOLD = 50
RIGHT_THRESHOLD = 50
MIN_LANDMINE_AREA = 180
LANDMINE_AREA_RATIO_THRESHOLD = 0.6
LINE_AREA_RATIO_THRESHOLD = 0.2
BRINK_RATIO_THREHOLD = 0.15
MIN_COLORED_LIGHT_AREA = 5000

# 颜色的字典
COLOR_RANGE = {'red': [(0, 43, 46), (6, 255, 255)],
               'green': [(54, 43, 46), (77, 255, 255)],
               'yellow': [(30, 43, 46), (50, 255, 255)],
               'black': [(0, 0, 0), (255, 255, 6)]
               }

RANGE_RGB = {'red': (0, 0, 255),
             'green': (0, 255, 0),
             'yellow': (255, 255, 0),
             'black': (0, 0, 0)
             }


# %% 图像处理
class ImageProcessor(object):
    def __init__(self, stream, debug):
        self._cap = cv2.VideoCapture(stream)
        self._debug = debug
        self._disposed = False
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self._cap.isOpened():
            ret, self._frame = self._cap.read()
            self._disposing = threading.Event()
            self._cap_thread = threading.Thread(target=self._get_frame)
            self._cap_thread.setDaemon(True)
            self._cap_thread.start()
        else:
            raise RuntimeError('Cannot open camera')

    def __del__(self):
        if not self._disposed:
            self._disposing.set()
            self._cap.release()
            if self._debug:
                cv2.destroyAllWindows()

    def _get_frame(self):
        while not self._disposing.is_set() and self._cap.isOpened():
            ret, self._frame = self._cap.read()
            time.sleep(0.01)

    # 函数定义
    # %% 判断黑色物体类型
    def _determine_object_type(self, contour):
        # 轮廓面积与最小外接圆面积之比
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(contour)
        area_ratio = cv2.contourArea(contour) / (math.pi * radius ** 2)
        if self._debug:
            print(area_ratio)

        if area_ratio >= LANDMINE_AREA_RATIO_THRESHOLD:
            return 'landmine'
        elif area_ratio < LINE_AREA_RATIO_THRESHOLD:
            return 'brink'
        else:
            return 'unrecognized'

    # %% 图像处理
    def _get_contours(self, img):
        # 摄像头默认分辨率 640x480,
        # 处理图像时会相应的缩小图像进行处理，这样可以加快运行速度
        # 缩小时保持比例4：3, 且缩小后的分辨率应该是整数
        img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT),
                         interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(img, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到 HSV 空间

        # 分离出各个 HSV 通道
        h, s, v = cv2.split(frame)
        # 直方图化
        v = cv2.equalizeHist(v)
        # 合并三个通道
        merged_frame = cv2.merge((h, s, v))

        # %% 寻找颜色轮廓
        img_contour = img
        contours = {}
        for i in COLOR_RANGE:
            frame = cv2.inRange(
                merged_frame, COLOR_RANGE[i][0], COLOR_RANGE[i][1])  # 对原图像和掩模进行位运算
            opened = cv2.morphologyEx(
                frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            closed = cv2.morphologyEx(
                opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
            # 注意此处 opencv 2 的返回值是三元元组
            (image, contours[i], hierarchy) = cv2.findContours(
                closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

            # %% 取面积高于阈值的轮廓
            area_threshold = MIN_LANDMINE_AREA if i == 'black' else MIN_COLORED_LIGHT_AREA
            contours[i] = list(filter(
                lambda x: cv2.contourArea(x) > area_threshold,
                contours[i]
            ))

            # %% 显示轮廓
            if self._debug:
                img_contour = cv2.drawContours(
                    img_contour, contours[i], -1, (255, 255, 255), 2)

        if self._debug:
            cv2.imshow("orgframe", img_contour)
            cv2.waitKey(1)

        return contours

    def get_objects_info(self):
        contours = self._get_contours(self._frame)
        info = {}
        info['landmine'] = []
        info['brink'] = []
        for contour in contours['black']:
            object_type = self._determine_object_type(contour)
            if object_type == 'landmine':
                bottom_most = tuple(contour[contour[:, :, 1].argmax()][0])
                info['landmine'].append(bottom_most)
            elif object_type == 'brink':
                [vx, vy, x, y] = cv2.fitLine(
                    contour, cv2.DIST_L2, 0, 0.01, 0.01)
                x_brink = x + (FRONT_THRESHOLD - y) * vx / vy
                if x_brink > 0:
                    info['brink'].append((x_brink, FRONT_THRESHOLD))

        info['light'] = []
        for i in contours:
            if self._debug:
                print(i + ' ' + str(len(contours[i])))

            if (len(contours[i]) > 0 and i != 'black'):
                info['light'].append(i)

        return info

    def dispose(self):
        self.disposed = True
        self.__del__()
