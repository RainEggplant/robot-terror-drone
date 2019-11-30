# -*- coding: utf-8 -*-
import math
import threading
import time
import numpy as np
import cv2

# %% 常数定义
USE_CLAHE = False
IMG_WIDTH = 320
IMG_HEIGHT = 240
FRONT_THRESHOLD = 200
LEFT_THRESHOLD = 50
RIGHT_THRESHOLD = 50
LANDMINE_SOLIDITY_THRESHOLD = 0.8
LANDMINE_AREA_RATIO_THRESHOLD = 0.6
BRINK_SOLIDITY_THREHOLD = 0.45
BRINK_AREA_RATIO_THREHOLD = 0.25
MIN_CONTOUR_AREA = {'white': 10000, 'red': 5000,
                    'green': 5000, 'yellow': 5000, 'black': 180}
MAX_TRACK_BRINK_DISTANCE = 3

# 检视平面坐标点
XT_LEFT = int(0.2 * IMG_WIDTH)
XT_MID = int(0.5 * IMG_WIDTH)
XT_RIGHT = int(0.8 * IMG_WIDTH)
YT_BOTTOM = int(0.9 * IMG_HEIGHT)
YT_MID = int(0.75 * IMG_HEIGHT)
YT_TOP = int(0.6 * IMG_HEIGHT)
TEST_POINTS = [(XT_LEFT, YT_BOTTOM), (XT_LEFT, YT_MID), (XT_LEFT, YT_TOP),
               (XT_MID, YT_BOTTOM), (XT_MID, YT_MID), (XT_MID, YT_TOP),
               (XT_RIGHT, YT_BOTTOM), (XT_RIGHT, YT_MID), (XT_RIGHT, YT_TOP)]

# 颜色字典
COLOR_RANGE = {
    'white': [(0, 0, 128), (255, 20, 255)],
    'red': [(0, 130, 165), (10, 255, 255)],
    'green': [(67, 114, 140), (104, 255, 255)],
    'yellow': [(30, 90, 186), (42, 255, 255)],
    'black': [(0, 0, 0), (255, 255, 76)]
}

COLOR_BGR = {
    'white': (255, 255, 255),
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'black': (0, 0, 0)
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
        # self.monitor = img
        img_gaussian = cv2.GaussianBlur(img, (3, 3), 0)  # 高斯模糊
        img_hsv = cv2.cvtColor(img_gaussian, cv2.COLOR_BGR2HSV)

        if USE_CLAHE:
            # CLAHE
            # equalizeHist is S**T!!!
            h, s, v = cv2.split(img_hsv)  # 分离出各个 HSV 通道
            v_max = np.amax(v)
            v_min = np.amin(v)
            v = ((v-v_min)/(v_max - v_min)) * 255
            v = v.astype('uint8')
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            v = clahe.apply(v)
            img_clahe = cv2.merge((h, s, v))  # 合并三个通道
            img_proc = img_clahe
            self.monitor = cv2.cvtColor(img_clahe, cv2.COLOR_HSV2BGR)
        else:
            img_proc = img_hsv
            self.monitor = img_gaussian

        # %% 寻找颜色轮廓
        contours = {}
        for i in COLOR_RANGE:
            mask_color = cv2.inRange(
                img_proc, COLOR_RANGE[i][0], COLOR_RANGE[i][1])  # 对原图像和掩模进行位运算
            mask_color = cv2.morphologyEx(
                mask_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            mask_color = cv2.morphologyEx(
                mask_color, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 闭运算
            # 注意此处 opencv 2 的返回值是三元元组

            if self._cv_version == '3':
                (_, contours[i], _) = cv2.findContours(
                    mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            else:
                (contours[i], _) = cv2.findContours(
                    mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

            # %% 取面积高于阈值的轮廓
            contours[i] = list(filter(
                lambda x: cv2.contourArea(x) > MIN_CONTOUR_AREA[i], contours[i]))

        return contours

    def _refresh_monitor(self):
        cv2.imshow("Monitor", self.monitor)
        cv2.waitKey(1)

    def analyze_objects(self):
        current_frame = self._frame
        contours = self._get_contours(current_frame)
        monitor = self.monitor
        rows, cols = monitor.shape[:2]  # 图片大小
        # 画出警戒线
        monitor = cv2.line(monitor, (0, FRONT_THRESHOLD),
                           (cols - 1, FRONT_THRESHOLD), COLOR_BGR['green'], 2)

        info = {}
        info['light'] = []

        # 首先判断信号灯
        keys_light = {'red', 'green', 'yellow'}
        contours_light = {key: value for key,
                          value in contours.items() if key in keys_light}
        for i in contours_light:
            if (len(contours_light[i]) > 0):
                info['light'].append(i)

        # 然后识别赛道平面
        max_contained_points = 0
        cnt_area = 0
        cnt_track = None
        for contour in contours['white']:
            contained_points = 0
            for point in TEST_POINTS:
                if cv2.pointPolygonTest(contour, point, False) != -1:
                    contained_points += 1

            area = cv2.contourArea(contour)
            if self._debug:
                print('Candidate track: ', contained_points, area)

            if (contained_points > max_contained_points or
                    (contained_points == max_contained_points and area >= cnt_area)):
                max_contained_points = contained_points
                cnt_area = area
                cnt_track = contour

        # 如果无法识别到赛道平面，直接返回
        if cnt_track is None:
            self.monitor = monitor
            if self._debug:
                self._refresh_monitor()
            return info

        # 用多边形近似平面
        epsilon = 0.01 * cv2.arcLength(cnt_track, True)
        cnt_track_approx = cv2.approxPolyDP(cnt_track, epsilon, True)
        info['track'] = cnt_track_approx
        monitor = cv2.drawContours(
            monitor, [cnt_track_approx], -1, COLOR_BGR['yellow'], 3)

        # 在赛道平面上识别地雷和边缘
        info['landmine'] = []
        info['brink'] = []
        for contour in contours['black']:
            (solidity, area_ratio) = self._get_area_ratio(contour)
            object_type = self._determine_object_type(
                'black', solidity, area_ratio)
            if object_type == 'landmine':
                bottom_most = tuple(contour[contour[:, :, 1].argmax()][0])
                # 只有在赛道平面内识别到的地雷才有效
                if cv2.pointPolygonTest(contour, bottom_most, False) != -1:
                    monitor = cv2.drawContours(
                        monitor, contour, -1, COLOR_BGR['red'], 2)
                    # monitor = cv2.putText(monitor, str(round(
                    #     area_ratio, 3)), bottom_most, cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR['white'], 1)
                    info['landmine'].append(bottom_most)
                if self._debug:
                    print('lamdmine: ', solidity, area_ratio)

            elif object_type == 'brink':
                # compute the center of the contour
                moments = cv2.moments(contour)
                x_c = int(moments['m10'] / moments['m00'])
                y_c = int(moments['m01'] / moments['m00'])
                distance = cv2.pointPolygonTest(contour, (x_c, y_c), True)
                # 识别到的边缘的中心必须与赛道边缘不能过远
                if distance < MAX_TRACK_BRINK_DISTANCE:
                    [vx, vy, x, y] = cv2.fitLine(
                        contour, cv2.DIST_L2, 0, 0.01, 0.01)
                    info['brink'].append((vx[0], vy[0], x[0], y[0]))

                    y_left = int((-x * vy / vx) + y)
                    y_right = int(((cols - x) * vy / vx) + y)
                    monitor = cv2.drawContours(
                        monitor, contour, -1, COLOR_BGR['white'], 2)
                    monitor = cv2.line(monitor, (cols - 1, y_right),
                                       (0, y_left), COLOR_BGR['red'], 2)
                    if self._debug:
                        print('brink: ', solidity, area_ratio, distance)
            else:
                if self._debug:
                    print('unknown', solidity, area_ratio)

        self.monitor = monitor
        if self._debug:
            self._refresh_monitor()

        return info

    # 终止摄像头线程，销毁对象
    def dispose(self):
        self._disposed = True
        self.__del__()
