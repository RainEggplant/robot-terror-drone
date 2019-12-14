# -*- coding: utf-8 -*-
import math
import threading
import time
import numpy as np
import cv2

# %% 常数定义
CV_VERSION = cv2.__version__.split('.')[0]  # pylint: disable=no-member
USE_CLAHE = True
IMG_WIDTH = 320
IMG_HEIGHT = 240
FRONT_THRESHOLD = 200
LEFT_THRESHOLD = 50
RIGHT_THRESHOLD = 50
LANDMINE_SOLIDITY_THRESHOLD = 0.8
LANDMINE_AREA_RATIO_THRESHOLD = 0.6
BRINK_SOLIDITY_THRESHOLD = 0.5
BRINK_AREA_THRESHOLD = 300
BRINK_AREA_RATIO_THRESHOLD = 0.05
BRINK_ANGLE_DELTA = 45
POSSIBLE_TRACK_SOLIDITY_THRESHOLD = 0.5
CANDIDATE_TRACK_SOLIDITY_THRESHOLD = 0.8
CANNY_MIN_THRESHOLD = 50
CANNY_MAX_THRESHOLD = 400
TOP_LINE = int(0.01 * IMG_HEIGHT)
BOTTOM_LINE = int(0.99 * IMG_HEIGHT)
MIN_CONTOUR_AREA = {'white': 2000, 'red': 5000,
                    'green': 5000, 'yellow': 5000, 'black': 150}
MIN_TRACK_BRINK_DISTANCE = -25

# 检视平面坐标点
XT_SCAN = np.arange(0.1 * IMG_WIDTH, IMG_WIDTH, 0.1 * IMG_WIDTH).astype(int)
YT_TOP = int(0.01 * IMG_HEIGHT)
YT_BOTTOM = int(0.99 * IMG_HEIGHT)

# 颜色字典
COLOR_RANGE = {
    'white': [(0, 0, 115), (255, 30, 255)],
    'red': [(0, 130, 165), (10, 255, 255)],
    'green': [(67, 114, 140), (104, 255, 255)],
    'yellow': [(30, 90, 186), (42, 255, 255)],
    'black': [(0, 0, 0), (255, 255, 68)]
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
        self._refresh = False
        self.monitor = None
        # self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self._cap.isOpened():
            ret, self._frame = self._cap.read()
            self._disposing = threading.Event()
            self._cap_thread = threading.Thread(target=self._get_frame)
            self._cap_thread.setDaemon(True)
            self._cap_thread.start()
            self._monitor_thread = threading.Thread(
                target=self._refresh_monitor)
            self._monitor_thread.setDaemon(True)
            self._monitor_thread.start()
        else:
            raise RuntimeError('Cannot open camera')

    def __del__(self):
        if not self._disposed:
            self._disposing.set()
            self._cap.release()
            cv2.destroyAllWindows()

    def _get_frame(self):
        while not self._disposing.is_set() and self._cap.isOpened():
            _, self._frame = self._cap.read()
            time.sleep(0.005)

    def _refresh_monitor(self):
        while not self._disposing.is_set() and self._cap.isOpened():
            if self._refresh:
                self._refresh = False
                cv2.imshow("Monitor", self.monitor)
                cv2.waitKey(1)
                time.sleep(0.005)

    @staticmethod
    def _preprocess_image(image):
        # 摄像头默认分辨率 640x480,
        # 处理图像时会相应的缩小图像进行处理，这样可以加快运行速度
        # 缩小时保持比例4：3, 且缩小后的分辨率应该是整数
        img_bgr = cv2.resize(image, (IMG_WIDTH, IMG_HEIGHT),
                             interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        # img_bgr = cv2.GaussianBlur(img_bgr, (3, 3), 0)  # 高斯模糊
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        if USE_CLAHE:
            # CLAHE
            # equalizeHist is S**T!!!
            h, s, v = cv2.split(img_hsv)  # 分离出各个 HSV 通道
            # 不能做归一化处理，否则容易放大噪声
            # v_max = np.amax(v)
            # v_min = np.amin(v)
            # v = ((v-v_min)/(v_max - v_min)) * 255
            # v = v.astype('uint8')
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            v = clahe.apply(v)
            img_hsv = cv2.merge((h, s, v))  # 合并三个通道
            img_bgr = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

        return img_bgr, img_hsv

    @staticmethod
    def _get_contours(img):
        # %% 寻找颜色轮廓
        contours = {}
        for i in COLOR_RANGE:
            mask_color = cv2.inRange(
                img, COLOR_RANGE[i][0], COLOR_RANGE[i][1])  # 对原图像和掩模进行位运算
            mask_color = cv2.morphologyEx(
                mask_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            mask_color = cv2.morphologyEx(
                mask_color, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 闭运算

            # 注意此处 OpenCV 3 的返回值是三元元组
            if CV_VERSION == '3':
                (_, contours[i], _) = cv2.findContours(
                    mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            else:
                (contours[i], _) = cv2.findContours(
                    mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

            # 取面积高于阈值的轮廓
            contours[i] = list(filter(
                lambda x: cv2.contourArea(x) > MIN_CONTOUR_AREA[i], contours[i]))

        return contours

    def _determine_object_type(self, contour, color):
        area = cv2.contourArea(contour)
        # 轮廓面积与凸包的比，因为不规则图形一般是噪声
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area
        area_ratio_circle = None
        area_ratio_ellipse = None
        angle = None

        if color == 'black':
            if solidity >= LANDMINE_SOLIDITY_THRESHOLD:
                # 轮廓面积与最小外接圆面积之比
                (_, radius) = cv2.minEnclosingCircle(contour)
                area_ratio_circle = area / (math.pi * radius ** 2)
                if area_ratio_circle >= LANDMINE_AREA_RATIO_THRESHOLD:
                    if self._debug:
                        print('landmine: area {}, solidity {:.3f}, area_ratio {:.3f}'.format(
                            area, solidity, area_ratio_circle))
                    return 'landmine'
            if solidity >= BRINK_SOLIDITY_THRESHOLD and area > BRINK_AREA_THRESHOLD:
                center, axes_len, angle = cv2.fitEllipse(contour)
                area_ratio_ellipse = area / \
                    (math.pi * axes_len[0] * axes_len[1] / 4)

                # 只选取角度接近水平的边缘
                if (area_ratio_ellipse >= BRINK_AREA_RATIO_THRESHOLD and
                        90 - BRINK_ANGLE_DELTA < angle < 90 + BRINK_ANGLE_DELTA):
                    if self._debug:
                        print('brink: area {}, solidity {:.3f}, area_ratio_ellipse {:.3f}, angle {:.3f}'.format(
                            area, solidity, area_ratio_ellipse, angle))
                    return 'brink'

            if self._debug:
                prompt_unknown = 'unknown: area {}, solidity {:.3f}'.format(
                    area, solidity)
                if area_ratio_circle is not None:
                    prompt_unknown = prompt_unknown + \
                        ', area_ratio_circle {:.3f}'.format(area_ratio_circle)
                if area_ratio_ellipse is not None:
                    prompt_unknown = prompt_unknown + \
                        ', area_ratio_ellipse {:.3f}'.format(
                            area_ratio_ellipse)
                if angle is not None:
                    prompt_unknown = prompt_unknown + \
                        ', angle {:.3f}'.format(angle)
                print(prompt_unknown)

            return 'unknown'

    def _get_tracks(self, contours, strict=False):
        track_cur = None
        track_next = None
        max_area_cur = 0
        max_area_next = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area
            threshold = CANDIDATE_TRACK_SOLIDITY_THRESHOLD if strict else POSSIBLE_TRACK_SOLIDITY_THRESHOLD
            if solidity < threshold:
                continue

            is_cur_track = False
            is_next_track = False
            for xt in XT_SCAN:
                if cv2.pointPolygonTest(contour, (xt, YT_BOTTOM), False) != -1:
                    is_cur_track = True
                if cv2.pointPolygonTest(contour, (xt, YT_TOP), False) != -1:
                    is_next_track = True
            is_next_track = is_next_track and (not is_cur_track)
            if is_cur_track and area > max_area_cur:
                track_cur = contour
            if is_next_track and area > max_area_next:
                track_next = contour

            if self._debug:
                prompt = 'candidate ' if strict else 'possible '
                track_type = 'current' if is_cur_track else (
                    'next' if is_next_track else 'unknown')
                print(
                    prompt + 'track: area {}, solidity {}, {}'.format(area, solidity, track_type))

        return track_cur, track_next

    @staticmethod
    def _refine_track(img, track):
        # 标记图像中可能属于赛道的部分的位置
        mask_edge = np.zeros((IMG_HEIGHT, IMG_WIDTH))
        mask_edge = cv2.drawContours(mask_edge, [track], 0, 1, 1).astype(bool)
        mask_track = np.zeros((IMG_HEIGHT, IMG_WIDTH))
        mask_track = cv2.drawContours(
            mask_track, [track], 0, 1, cv2.FILLED).astype(bool)

        # 取得灰度图片，求取平均亮度及标准差
        img_track = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_track[~mask_track] = 0
        mask_canny = cv2.Canny(img_track, CANNY_MIN_THRESHOLD,
                               CANNY_MAX_THRESHOLD).astype(bool)
        mask_edge = np.logical_or(mask_edge, mask_canny).astype('uint8')

        # 再提取赛道形状
        # 注意此处 OpenCV 3 的返回值是三元元组
        if CV_VERSION == '3':
            (_, cnt_tracks, hierarchy) = cv2.findContours(
                mask_edge, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        else:
            (cnt_tracks, hierarchy) = cv2.findContours(
                mask_edge, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        for i in range(hierarchy[0].shape[0]):
            if hierarchy[0][i][2] != -1:
                cnt_tracks[i] = None

        cnt_tracks = list(filter(lambda x: x is not None, cnt_tracks))
        cnt_tracks = list(filter(
            lambda x: cv2.contourArea(x) > MIN_CONTOUR_AREA['white'], cnt_tracks))

        return cnt_tracks

    @staticmethod
    def _approx_track(track):
        """ 用多边形近似平面 """
        epsilon = 0.01 * cv2.arcLength(track, True)
        return cv2.approxPolyDP(track, epsilon, True)

    def analyze_objects(self):
        current_frame = self._frame
        img_bgr, img_hsv = self._preprocess_image(current_frame)
        contours = self._get_contours(img_hsv)
        monitor = img_bgr.copy()
        rows, cols = monitor.shape[:2]  # 图片大小
        # 画出警戒线
        monitor = cv2.line(monitor, (0, FRONT_THRESHOLD),
                           (cols - 1, FRONT_THRESHOLD), COLOR_BGR['green'], 2)

        info = dict()
        info['light'] = []
        # %% 识别信号灯
        keys_light = {'red', 'green', 'yellow'}
        contours_light = {key: value for key,
                          value in contours.items() if key in keys_light}
        for i in contours_light:
            if len(contours_light[i]) > 0:
                info['light'].append(i)

        # %% 识别赛道平面
        track_cur, track_next = self._get_tracks(contours['white'], False)

        # %% 对提取的赛道进行二次处理
        if track_cur is not None:
            tracks = self._refine_track(img_bgr, track_cur)
            track_cur, _ = self._get_tracks(tracks, True)
        if track_next is not None:
            tracks = self._refine_track(img_bgr, track_next)
            _, track_next = self._get_tracks(tracks, True)

        if track_cur is not None:
            track_cur_approx = self._approx_track(track_cur)
            info['track'] = track_cur_approx
            monitor = cv2.drawContours(
                monitor, [track_cur_approx], -1, COLOR_BGR['yellow'], 3)

        if track_next is not None:
            track_next_approx = self._approx_track(track_next)
            info['track_next'] = track_next_approx
            monitor = cv2.drawContours(
                monitor, [track_next_approx], -1, COLOR_BGR['yellow'], 3)

        # %% 识别地雷和边缘
        brink_cur = []
        brink_next = []
        info['landmine'] = []
        for contour in contours['black']:
            object_type = self._determine_object_type(contour, 'black')
            if track_cur is not None and object_type == 'landmine':
                bottom_most = tuple(contour[contour[:, :, 1].argmax()][0])
                # 只有在赛道平面内识别到的地雷才有效
                if cv2.pointPolygonTest(track_cur, bottom_most, False) != -1:
                    monitor = cv2.drawContours(
                        monitor, contour, -1, COLOR_BGR['red'], 2)
                    info['landmine'].append(bottom_most)

            elif object_type == 'brink':
                ellipse = cv2.fitEllipse(contour)
                center, axes_len, angle = ellipse
                long_axis = max(axes_len)
                angle_rad = angle / 180 * math.pi
                vx, vy = math.sin(angle_rad), -math.cos(angle_rad)
                t_x1 = -center[0] / vx
                t_x2 = (IMG_WIDTH - 1 - center[0]) / vx
                if t_x1 > t_x2:
                    t_x1, t_x2 = t_x2, t_x1

                if angle == 90:
                    t_y1 = -long_axis / 2
                    t_y2 = t_y1
                else:
                    t_y1 = -center[1] / vy
                    t_y2 = (IMG_HEIGHT - 1 - center[1]) / vy
                    if t_y1 > t_y2:
                        t_y1, t_y2 = t_y2, t_y1

                t1 = max(t_x1, t_y1, -long_axis / 2)
                t2 = min(t_x2, t_y2, long_axis / 2)
                t_center = (t1 + t2) / 2
                x = center[0] + t_center * vx
                y = center[1] + t_center * vy

                if track_cur is not None:
                    distance_cur = cv2.pointPolygonTest(
                        track_cur, (x, y), True)
                else:
                    distance_cur = y - (IMG_HEIGHT - 1)

                if track_next is not None:
                    distance_next = cv2.pointPolygonTest(
                        track_next, (x, y), True)
                else:
                    distance_next = -y

                if distance_cur > MIN_TRACK_BRINK_DISTANCE:
                    brink_cur.append(ellipse)
                elif distance_next > MIN_TRACK_BRINK_DISTANCE:
                    brink_next.append(ellipse)

        # %% 判断沟壑
        if ((track_cur is not None or track_next is not None) and
                len(brink_cur) > 0 and len(brink_next) > 0):
            elp_cur = max(brink_cur, key=lambda e: e[1][0] * e[1][1])
            elp_next = max(brink_next, key=lambda e: e[1][0] * e[1][1])
            ditch = [elp_cur, elp_next]
            monitor = cv2.ellipse(monitor, elp_cur, (0, 0, 255), 2)
            monitor = cv2.ellipse(monitor, elp_next, (0, 0, 255), 2)
            info['ditch'] = ditch

        # 判断开合门
        info['block'] = False
        # 检测图像内是否有开合门。有开合门则返回包含开合门位置的矩形框，否则返回空元组
        # 返回为三元tuple：((x, y), (w, h), angle)
        # (x, y)为中心坐标，(w, h)为矩形宽高，angle为矩形相对水平位置的倾斜角度，范围为-90~90
        img_h, img_w = rows, cols
        img_yellow = np.where((img_hsv[:, :, 0] >= 10) & (img_hsv[:, :, 0] <= 35) & (
            img_hsv[:, :, 1] >= 130) & (img_hsv[:, :, 2] >= 130), 255, 0)
        img_yellow = np.array(img_yellow, np.uint8)
        if CV_VERSION == '3':
            (_, img_yellow_cnts, _) = cv2.findContours(
                img_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            (img_yellow_cnts, _) = cv2.findContours(
                img_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnt_rect = []
        road_block = ()
        # 如果完全没有找到黄色Contour，直接返回空tuple
        if img_yellow_cnts:
            for cnt in img_yellow_cnts:
                # 去除面积过小的Contour
                if math.fabs(cv2.contourArea(cnt)) <= 2.5e-4 * img_h * img_w:
                    continue
                cnt_min_rect = cv2.minAreaRect(cnt)
                cnt_box = cv2.boxPoints(cnt_min_rect)
                cnt_box = np.int0(cnt_box)
                cnt_rect.append(cnt_box)

        road_block_cnt = []
        # 如果没有足够大的黄色Contour，直接返回空tuple
        if cnt_rect:
            for cnt in cnt_rect:
                if np.all(cnt == cnt_rect[0]):
                    road_block_cnt = cnt
                else:
                    road_block_cnt = np.append(road_block_cnt, cnt, axis=0)
            road_block = cv2.minAreaRect(road_block_cnt)
            # road_block_box = cv2.boxPoints(road_block)
            # road_block_box = np.int0(road_block_box)
            # cv2.drawContours(img, cnt_rect, -1, (255, 0, 255), 2)
            # cv2.drawContours(img, [road_block_box], -1, (255, 255, 0), 2)

        if road_block:
            # 待调整
            if math.fabs(road_block[2]) <= 30:
                info['block'] = True

        self.monitor = monitor
        self._refresh = True

        return info

    def dispose(self):
        """ 终止摄像头线程，销毁对象 """
        self._disposed = True
        self.__del__()
