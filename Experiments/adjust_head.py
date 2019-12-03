import sys
import time
sys.path.append('..')  # nopep8
import Serial_Servo_Running as SSR
import PWMServo

PWMServo.setServo(1, 2150, 600)  #up and down up-little
time.sleep(0.8)
PWMServo.setServo(2, 1500, 600)  # Same as above
time.sleep(0.8)
