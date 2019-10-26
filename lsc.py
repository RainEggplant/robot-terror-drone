#!/usr/bin/python
# -*- coding: UTF-8 -*-

import SocketServer
import PWMServo
import Serial_Servo_Running as SSR
import threading

DEBUG = False
client_socket = []
inf_flag = False

# 启动线程运行动作组
SSR.start_action_thread()


class LobotServer(SocketServer.ThreadingTCPServer):
    allow_reuse_address = True  # 允许地址重用


class LobotServerHandler(SocketServer.BaseRequestHandler):
    global client_socket
    ip = ""
    port = None

    def setup(self):
        self.ip = self.client_address[0].strip()
        self.port = self.client_address[1]
        print("connected\tIP:"+self.ip+"\tPort:"+str(self.port))
        client_socket.append(self.request)  # 将此连接加入客户端列表
        self.request.settimeout(6)  # 超时时间为6秒

    def handle(self):
        global action_num, action_times, inf_flag
        conn = self.request
        recv_data = b''
        Flag = True
        while Flag:
            try:
                buf = conn.recv(1024)
                if buf == b'':
                    Flag = False
                else:
                    recv_data = recv_data + buf
                    # 解决断包问题，接收到完整命令后才发送到串口,防止出错
                    while True:
                        try:
                            index = recv_data.index(b'\x55\x55')  # 搜索数据中的0x55 0x55
                            if len(recv_data) >= index+3:  # 缓存中的数据长度是否足够
                                recv_data = recv_data[index:]
                                if ord(recv_data[2]) + 2 <= len(recv_data):  # 缓存中的数据长度是否足够
                                    cmd = recv_data[0:ord(recv_data[2])+2]    # 取出命令
                                    recv_data = recv_data[ord(recv_data[2])+3:]  # 去除已经取出的命令
                                    if ord(cmd[0]) and ord(cmd[1]) is 0x55:
                                        if ord(cmd[2]) == 0x08 and ord(cmd[3]) == 0x03:  # 数据长度和控制单舵机命令
                                            print('id', ord(cmd[7]))
                                            id = ord(cmd[7])
                                            pos = 0xffff & ord(cmd[8]) | 0xff00 & (ord(cmd[9]) << 8)
                                            print('pos', pos)
                                            if id == 6:
                                                PWMServo.setServo(1, pos, 20)
                                            elif id == 7:
                                                PWMServo.setServo(2, pos, 20)
                                            else:
                                                pass
                                        elif ord(cmd[2]) == 0x05 and ord(cmd[3]) == 0x06:
                                            action_num = ord(cmd[4])
                                            action_times = 0xffff & ord(cmd[5]) | 0xff00 & (ord(cmd[6]) << 8)
                                            print 'action', action_num
                                            print 'times', action_times
                                            if action_times == 0:  # 无限次
                                                SSR.change_action_value(str(action_num), action_times)
                                                inf_flag = True
                                            else:
                                                if inf_flag:
                                                    SSR.stop_action_group()
                                                    inf_flag = False
                                                else:
                                                    SSR.change_action_value(str(action_num), action_times)

                                    if DEBUG is True:
                                        for i in cmd:
                                            print hex(ord(i))
                                        print('*' * 30)
                                else:
                                    break
                            else:
                                break
                        except Exception as e:   # 在recv_data 中搜不到 '\x55\x55'子串
                            break
                    recv_data = b''
                    action_times = None
                    action_num = None
            except Exception as e:
                print(e)
                Flag = False
                break

    def finish(self):
        client_socket.remove(self.request)  # 从客户端列表中剔除此连接
        print("disconnected\tIP:"+self.ip+"\tPort:"+str(self.port))


if __name__ == "__main__":
    PWMServo.setServo(1, 1500, 1000)
    PWMServo.setServo(2, 1500, 1000)
    server = LobotServer(("", 9029), LobotServerHandler)    # 建立服务器
    try:
        server.serve_forever()  # 开始服务器循环
    except Exception as e:
        print(e)
        server.shutdown()
        server.server_close()

