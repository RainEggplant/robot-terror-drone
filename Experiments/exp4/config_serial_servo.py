#!/usr/bin/python3
# encoding: utf-8
# 配置串口舵机的参数
# 每次只能配置一个舵机，且树莓派扩展板只能连接一个舵机，既是一个舵机一个舵机配置参数

from SerialServoCmd import *


def serial_servo_set_id(oldid, newid):
    """
    配置舵机id号, 出厂默认为1
    :param oldid: 原来的id， 出厂默认为1
    :param newid: 新的id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)


def serial_servo_read_id(id=None):
    """
    读取串口舵机id
    :param id: 默认为空
    :return: 返回舵机id
    """
    while True:
        if id is None:  # 总线上只能有一个舵机
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # 获取内容
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg


def serial_servo_stop(id=None):
    '''
    停止舵机运行
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)


def serial_servo_set_deviation(id, d=0):
    """
    配置偏差，掉电保护
    :param id: 舵机id
    :param d:  偏差
    """
    # 设置偏差
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)
    # 设置为掉电保护
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)


def serial_servo_read_deviation(id):
    '''
    读取偏差值
    :param id: 舵机号
    :return:
    '''
    # 发送读取偏差指令
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # 获取
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        if msg is not None:
            return msg


def serial_servo_set_angle_limit(id, low, high):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)


def serial_servo_read_angle_limit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            return msg


def serial_servo_set_vin_limit(id, low, high):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)


def serial_servo_read_vin_limit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg


def serial_servo_set_max_temp(id, m_temp):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)


def serial_servo_read_temp_limit(id):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg


def serial_servo_read_pos(id):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg


def serial_servo_read_temp(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg


def serial_servo_read_vin(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg


def serial_servo_rest_pos(oldid):
    # 舵机清零偏差和P值中位（500）
    serial_servo_set_deviation(oldid, 0)    # 清零偏差
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # 中位


def show_servo_state():
    '''
    显示信息
    :return:
    '''
    oldid = serial_servo_read_id()
    portRest()
    if oldid is not None:
        print('当前的舵机ID是：%d' % oldid)
        pos = serial_servo_read_pos(oldid)
        print('当前的舵机角度：%d' % pos)
        portRest()

        now_temp = serial_servo_read_temp(oldid)
        print('当前的舵机温度：%d°' % now_temp)
        portRest()

        now_vin = serial_servo_read_vin(oldid)
        print('当前的舵机电压：%dmv' % now_vin)
        portRest()

        d = serial_servo_read_deviation(oldid)
        print('当前的舵机偏差：%d' % ctypes.c_int8(d).value)
        portRest()

        limit = serial_servo_read_angle_limit(oldid)
        print('当前的舵机可控角度为%d-%d' % (limit[0], limit[1]))
        portRest()

        vin = serial_servo_read_vin_limit(oldid)
        print('当前的舵机报警电压为%dmv-%dmv' % (vin[0], vin[1]))
        portRest()

        temp = serial_servo_read_temp_limit(oldid)
        print('当前的舵机报警温度为50°-%d°' % temp)
        portRest()


    return oldid


if __name__ == '__main__':
    portInit()
    oldid = show_servo_state()
    while True:
        print ('*' * 50)
        print ('1、设置舵机ID号')
        print ('2、设置舵机偏差')
        print ('3、设置舵机转动角度范围')
        print ('4、设置舵机电压报警范围')
        print ('5、设置舵机温度报警范围')
        print ('6、显示舵机状态')
        print ('7、中位舵机')
        print ('8、退出')
        print ('*' * 50)
        num = input('请输入需要配置的数字')
        num6_flag = 0
        while 1 <= num <= 7:
            if num == 1:
                num6_flag = 0
                n_id = input('请输入新的舵机ID号（范围：0-253）')
                if n_id > 253:
                    print ('超过范围，请重新输入')
                else:
                    serial_servo_set_id(oldid, n_id)
                    portRest()
                    if serial_servo_read_id() == n_id:
                        # 不成功继续
                        oldid = n_id
                        print ('设置成功')
                        break
            elif num == 2:
                num6_flag = 0
                n_d = input('请输入舵机偏差值（范围：-125 ~ 125）')
                if n_d < -125 or n_d > 125:
                    print ('超过范围，请重新输入')
                else:
                    serial_servo_set_deviation(oldid, n_d)
                    time.sleep(0.1)
                    portRest()
                    zf_d = serial_servo_read_deviation(oldid)
                    if zf_d > 127:  # 负数
                        zf_d = -(0xff - (zf_d - 1))
                    if zf_d == n_d:
                        print ('设置成功')
                        break
            elif num == 3:
                num6_flag = 0
                print ('请输入舵机的转动范围（0 ~ 1000）')
                low_ang_limit = input('请输入低范围值')
                high_ang_limit = input('请输入高范围值')
                if low_ang_limit < 0 or high_ang_limit < 0 or low_ang_limit >= 1000 or high_ang_limit > 1000:
                    print ('超过范围，请重新输入')
                else:
                    serial_servo_set_angle_limit(oldid, low_ang_limit, high_ang_limit)
                    portRest()
                    lim = serial_servo_read_angle_limit(oldid)
                    if lim[0] == low_ang_limit and lim[1] == high_ang_limit:
                        print ('设置成功')
                        break
            elif num == 4:
                num6_flag = 0
                print ('请输入舵机的电压报警范围（4500 ~ 12000）mv')
                low_vin_limit = input('请输入低范围值')
                high_vin_limit = input('请输入高范围值')
                if low_vin_limit < 4500 or high_vin_limit < 4500 or low_vin_limit >= 12000 or high_vin_limit > 12000:
                    print ('超过范围，请重新输入')
                else:
                    serial_servo_set_vin_limit(oldid, low_vin_limit, high_vin_limit)
                    portRest()
                    vin = serial_servo_read_vin_limit(oldid)
                    if vin[0] == low_vin_limit and vin[1] == high_vin_limit:
                        print ('设置成功')
                        break
            if num == 5:
                num6_flag = 0
                n_temp = input('请输入舵机的温度报警范围（范围：50-100）度')
                if n_temp > 100 or n_temp < 50:
                    print ('超过范围，请重新输入')
                else:
                    serial_servo_set_max_temp(oldid, n_temp)
                    portRest()
                    if serial_servo_read_temp_limit(oldid) == n_temp:
                        # 不成功继续
                        print ('设置成功')
                        break
            elif num == 6:
                if num6_flag == 0:
                    oldid = show_servo_state()
                    num6_flag = 1
                break
            elif num == 7:
                num6_flag = 0
                serial_servo_rest_pos(oldid)
                print ('中位成功')
                break
        if num == 8:
            break
        elif num < 1 or num > 8:
            print ('输入有误，请重新输入')
    # serial_servo_set_id(9, 20)
    # portRest()

    # serial_servo_set_deviation(20, 45)
    # portRest()
    # serial_servo_read_deviation(20)
    # time.sleep(1)



