import time
from adafruit_motor import servo
from adafruit_pca9685 import pca9685
from board import SCL,SDA
from math import pi

TO_DEGREE = 180.0/pi

class ServoDriver(object):
    """ServoDriver"""

    def __init__(self, n_joints, servo_freq):
        super(ServoDriver, self).__init__()

        self.N_JOINTS = n_joints
        i2c=busio.I2C(SCL,SDA)

        self.pca=PCA9685(i2c)
        self.servo_list=[
            servo_list.append(servo.Servo(pca.channels[0],min_pulse=700,max_pulse=2700)),
            servo_list.append(servo.Servo(pca.channels[1],min_pulse=600,max_pulse=2500)),
            servo_list.append(servo.Servo(pca.channels[2],min_pulse=600,max_pulse=2500)),
            servo_list.append(servo.Servo(pca.channels[3],min_pulse=400,max_pulse=2400)),
            servo_list.append(servo.Servo(pca.channels[4],min_pulse=450,max_pulse=2400)),
            servo_list.append(servo.Servo(pca.channels[5],min_pulse=450,max_pulse=2425))
            ]
        self.pca.frequency=servo_freq

    def write_position(self, theta):
        servo_list[0].angle = 90.0 + theta[0]*TO_DEGREE
        servo_list[1].angle = 90.0 - theta[1]*TO_DEGREE
        servo_list[2].angle = 90.0 - theta[2]*TO_DEGREE
        servo_list[3].angle = 90.0 + theta[3]*TO_DEGREE
        servo_list[4].angle = 90.0 - theta[4]*TO_DEGREE
        servo_list[5].angle = 90.0 + theta[5]*TO_DEGREE

    def execute_trajectory(self, trajectory):
        for position in trajectory:
            self.write_position(position)