#!/usr/bin/python3
#coding:utf8
import subprocess
import time
import threading
import signal
import pigpio

ChildCvLine = subprocess.Popen(["python3", "/home/pi/human_code/line_patrol.py"])

key1_pin = 25
key2_pin = 22
led2_pin = 23

key1 = pigpio.pi()
key2 = pigpio.pi()
led2 = pigpio.pi()

led2.set_mode(led2_pin, pigpio.OUTPUT)

def ledflash():   
    led2.write(led2_pin, 1)
    time.sleep(0.1)
    led2.write(led2_pin, 0)
    time.sleep(0.1)
    
def start():
    key1.set_mode(key1_pin, pigpio.INPUT)
    key1.set_pull_up_down(key1_pin, pigpio.PUD_UP)
    while True:
        if not key1.read(key1_pin) > 0:
            count += 1
        else:
            count = 0
        if count >= 1:
            #print("start")
            ledflash()
            ChildCvLine.send_signal(signal.SIGCONT)
        time.sleep(0.1)
       
def stop():
    key2.set_mode(key2_pin, pigpio.INPUT)
    key2.set_pull_up_down(key2_pin, pigpio.PUD_UP)
    while True:
        if not key2.read(key2_pin) > 0:
            count += 1
        else:
            count = 0
        if count >= 1:
            #print("stop")
            ledflash()
            ChildCvLine.send_signal(signal.SIGTSTP)
        time.sleep(0.1)

key1_th = threading.Thread(target = start).start()
key2_th = threading.Thread(target = stop).start()
