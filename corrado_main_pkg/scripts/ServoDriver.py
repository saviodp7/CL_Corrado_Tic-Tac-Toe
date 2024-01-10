import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from board import SCL,SDA
from math import pi
import busio

TO_DEGREE = 180.0/pi

class ServoDriver(object):
    """ServoDriver"""

    def __init__(self, n_joints, servo_freq):
        super(ServoDriver, self).__init__()

        self.N_JOINTS = n_joints
        i2c=busio.I2C(SCL,SDA)

        self.pca=PCA9685(i2c)
        self.pca.frequency=servo_freq
        self.servo_list=[]
        self.servo_list.append(servo.Servo(self.pca.channels[0],min_pulse=650,max_pulse=2650))
        self.servo_list.append(servo.Servo(self.pca.channels[1],min_pulse=550,max_pulse=2600))
        self.servo_list.append(servo.Servo(self.pca.channels[2],min_pulse=600,max_pulse=2600))
        self.servo_list.append(servo.Servo(self.pca.channels[3],min_pulse=400,max_pulse=2400))
        self.servo_list.append(servo.Servo(self.pca.channels[4],min_pulse=700,max_pulse=2850))
        self.servo_list.append(servo.Servo(self.pca.channels[5],min_pulse=520,max_pulse=2450))
        

    def write_position(self, theta):
        self.servo_list[0].angle = 90.0 + theta[0]*TO_DEGREE
        self.servo_list[1].angle = 90.0 - theta[1]*TO_DEGREE
        self.servo_list[2].angle = 90.0 - theta[2]*TO_DEGREE
        self.servo_list[3].angle = 90.0 + theta[3]*TO_DEGREE
        self.servo_list[4].angle = 90.0 - theta[4]*TO_DEGREE
        self.servo_list[5].angle = 90.0 + theta[5]*TO_DEGREE

    def execute_trajectory(self, trajectory):
        for position in trajectory:
            self.write_position(position)
            time.sleep(0.05)
