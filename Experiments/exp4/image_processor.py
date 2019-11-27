# -*- coding: utf-8 -*-
import math
import threading
import time
import numpy as np
import cv2

# %% 常数定义
IMG_WIDTH = 320
IMG_HEIGHT = 240
FRONT_THRESHOLD = 200
LEFT_THRESHOLD = 50
RIGHT_THRESHOLD = 50
MIN_LANDMINE_AREA = 180
LANDMINE_SOLIDITY_THRESHOLD = 0.8
LANDMINE_AREA_RATIO_THRESHOLD = 0.6
BRINK_SOLIDITY_THREHOLD = 0.45
BRINK_AREA_RATIO_THREHOLD = 0.25
MIN_COLORED_LIGHT_AREA = 5000

# 颜色的字典
COLOR_RANGE = {'red': [(0, 43, 46), (6, 255, 255)],
               'green': [(54, 43, 46), (77, 255, 255)],
               'yellow': [(30, 43, 46), (50, 255, 255)],
               'black': [(0, 0, 0), (255, 255, 10)]
               }

COLOR_RGB = {'red': (0, 0, 255),
             'green': (0, 255, 0),
             'yellow': (255, 255, 0),
             'black': (0, 0, 0),
             'white': (255, 255, 255)
             }


# %% 图像处理
class ImageProcessor(object):
    def __init__(self, stream, debug):
        self._cap = cv2.VideoCapture(stream)
        self._debug = debug
        self._disposed = False
        # check OpenCV version
        self._cv_version = cv2.__version__.split('.')[0]
        self.monitor = None
        # self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
            _, self._frame = self._cap.read()
            time.sleep(0.01)

    # %% 获得轮廓面积与凸包及最小外接圆面积之比
    def _get_area_ratio(self, contour):
        area = cv2.contourArea(contour)

        # 轮廓面积与凸包的比
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area

        # 轮廓面积与最小外接圆面积之比
        (_, radius) = cv2.minEnclosingCircle(contour)
        area_ratio = area / (math.pi * radius ** 2)

        return (solidity, area_ratio)

    def _determine_object_type(self, color, solidity, area_ratio):
        if color == 'black':
            if (solidity >= LANDMINE_SOLIDITY_THRESHOLD and
                    area_ratio >= LANDMINE_AREA_RATIO_THRESHOLD):
                return 'landmine'
            if (solidity >= BRINK_SOLIDITY_THREHOLD and
                    area_ratio <= BRINK_AREA_RATIO_THREHOLD):
                return 'brink'
        return 'unknown'

    def _get_contours(self, img):
        # 摄像头默认分辨率 640x480,
        # 处理图像时会相应的缩小图像进行处理，这样可以加快运行速度
        # 缩小时保持比例4：3, 且缩小后的分辨率应该是整数
        img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT),
                         interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        self.monitor = img
        frame = cv2.GaussianBlur(img, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到 HSV 空间

        # 分离出各个 HSV 通道
        h, s, v = cv2.split(frame)
        # 直方图化
        v = cv2.equalizeHist(v)
        # 合并三个通道
        merged_frame = cv2.merge((h, s, v))

        # %% 寻找颜色轮廓
        contours = {}
        for i in COLOR_RANGE:
            frame = cv2.inRange(
                merged_frame, COLOR_RANGE[i][0], COLOR_RANGE[i][1])  # 对原图像和掩模进行位运算
            opened = cv2.morphologyEx(
                frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            closed = cv2.morphologyEx(
                opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
            # 注意此处 opencv 2 的返回值是三元元组

            if self._cv_version == '3':
                (_, contours[i], _) = cv2.findContours(
                    closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            else:
                (contours[i], _) = cv2.findContours(
                    closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

            # %% 取面积高于阈值的轮廓
            area_threshold = MIN_LANDMINE_AREA if i == 'black' else MIN_COLORED_LIGHT_AREA
            contours[i] = list(filter(
                lambda x: cv2.contourArea(x) > area_threshold,
                contours[i]
            ))

        return contours

    def _refresh_monitor(self):
        cv2.imshow("orgframe", self.monitor)
        cv2.waitKey(1)

    def analyse_objects(self):
        current_frame = self._frame
        contours = self._get_contours(current_frame)
        monitor = self.monitor
        rows, cols = monitor.shape[:2]  # 图片大小
        # 画出警戒线
        monitor = cv2.line(monitor, (0, FRONT_THRESHOLD),
                           (cols - 1, FRONT_THRESHOLD), COLOR_RGB['green'], 2)

        info = {}
        info['landmine'] = []
        info['brink'] = []
        for contour in contours['black']:
            (solidity, area_ratio) = self._get_area_ratio(contour)
            object_type = self._determine_object_type(
                'black', solidity, area_ratio)
            if self._debug:
                print(object_type, solidity, area_ratio)

            if object_type == 'landmine':
                bottom_most = tuple(contour[contour[:, :, 1].argmax()][0])
                monitor = cv2.drawContours(
                    monitor, contour, -1, COLOR_RGB['red'], 2)
                # monitor = cv2.putText(monitor, str(round(
                #     area_ratio, 3)), bottom_most, cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_RGB['white'], 1)
                info['landmine'].append(bottom_most)
            elif object_type == 'brink':
                [vx, vy, x, y] = cv2.fitLine(
                    contour, cv2.DIST_L2, 0, 0.01, 0.01)
                x_brink = x + (FRONT_THRESHOLD - y) * vx / vy
                if x_brink > 0:
                    info['brink'].append((x_brink[0], FRONT_THRESHOLD))

                y_left = int((-x * vy / vx) + y)
                y_right = int(((cols - x) * vy / vx) + y)
                monitor = cv2.drawContours(
                    monitor, contour, -1, COLOR_RGB['white'], 2)
                monitor = cv2.line(monitor, (cols - 1, y_right),
                                   (0, y_left), COLOR_RGB['red'], 2)

        info['light'] = []
        for i in contours:
            if (len(contours[i]) > 0 and i != 'black'):
                info['light'].append(i)

        self.monitor = monitor
        if self._debug:
            self._refresh_monitor()

        return info

    # 终止摄像头线程，销毁对象
    def dispose(self):
        self._disposed = True
        self.__del__()
