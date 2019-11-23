#!/usr/bin/env python3
# encoding: utf-8
import time
import os
import sqlite3 as sql
import SerialServoCmd as ssc
import config_serial_servo
import threading


runningAction = False
stopRunning = False


def serial_setServo(s_id, pos, s_time):
    if pos > 1000:
        pos = 1000
    elif pos < 0:
        pos = 0
    else:
        pass
    if s_time > 30000:
        s_time = 30000
    elif s_time < 10:
        s_time = 10
    ssc.serial_serro_wirte_cmd(s_id, ssc.LOBOT_SERVO_MOVE_TIME_WRITE, pos, s_time)


def setDeviation(servoId, d):
    '''
    配置舵机偏差
    :param servoId:
    :param d:
    :return:
    '''
    global runningAction
    if servoId < 1 or servoId > 16:
        return
    if d < -200 or d > 200:
        return
    if runningAction is False:
        config_serial_servo.serial_servo_set_deviation(servoId, d)


def stop_action_group():
    global stopRunning
    stopRunning = True


def running_action_group(actNum, times):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param times:  运行次数
    :return:
    '''
    global runningAction
    global stopRunning
    actNum = "/home/pi/human_code/ActionGroups/" + actNum + ".d6a"
    while times:    # 运行次数
        if os.path.exists(actNum) is True:
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            if runningAction is False:
                runningAction = True
                while True:
                    act = cu.fetchone()
                    if stopRunning is True:
                        print('stop')
                        stopRunning = False
                        runningAction = False
                        cu.close()
                        ag.close()
                        break
                    if act is not None:
                        for i in range(0, len(act)-2, 1):
                                serial_setServo(i+1, act[2 + i], act[1])
                        time.sleep(float(act[1]*1.1)/1000.0)
                    else:
                        runningAction = False
                        cu.close()
                        ag.close()
                        break
        else:
            runningAction = False
            print("未能找到动作组文件")
        times -= 1


def runAction(actNum):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param times:  运行次数
    :return:
    '''
    global runningAction
    global stopRunning
    global online_action_times
    if actNum is None:
        return
    actNum = "/home/pi/human_code/ActionGroups/" + actNum + ".d6a"
    if os.path.exists(actNum) is True:
        ag = sql.connect(actNum)
        cu = ag.cursor()
        cu.execute("select * from ActionGroup")
        if runningAction is False:
            runningAction = True
            while True:
                act = cu.fetchone()
                if stopRunning is True:
                    print ('stop')
                    stopRunning = False
                    runningAction = False
                    cu.close()
                    ag.close()
                    break
                if online_action_times == -1:   # 切换动作组
                    stopRunning = False
                    runningAction = False
                    cu.close()
                    ag.close()
                    break
                if act is not None:
                    for i in range(0, len(act)-2, 1):
                        serial_setServo(i+1, act[2 + i], act[1])
                    time.sleep(float(act[1]*1.1)/1000.0)
                else:   # 运行完才退出
                    runningAction = False
                    cu.close()
                    ag.close()
                    break
    else:
        runningAction = False
        print("未能找到动作组文件")


def online_thread_run_acting():
    global online_action_times, online_action_num
    while True:
        if online_action_times == 0:
            # 无限次运行
            runAction(online_action_num)
        elif online_action_times > 0:
            # 有次数运行
            runAction(online_action_num)
            online_action_times = -1    # 运行完成后，进入空载
        else:
            # 空载
            time.sleep(0.01)


def stop_servo():
    print("停止机械臂")
    for i in range(16):
        config_serial_servo.serial_servo_stop(i+1)


def start_action_thread():
    th1 = threading.Thread(target=online_thread_run_acting)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th1.start()


def change_action_value(actNum, actTimes):
    global online_action_times, online_action_num
    online_action_times = actTimes
    online_action_num = actNum
