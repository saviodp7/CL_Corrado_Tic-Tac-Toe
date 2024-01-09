import time
from adafruit_motor import servo
from adafruit_pca9685 import pca9685
from board import SCL,SDA

N_JOINTS = 6
F_POSITIONS = 200
SERVO_FREQ = 200

i2c=busio.I2C(SCL,SDA)

pca=PCA9685(i2c)
pca.frequency=SERVO_FREQ

def homing():
    home_position = [90,90,90,90,90,90]
    for i, theta in enumerate(home_position):
        servo_list[i].angle=theta
    time.sleep(1)

trajectory=list()

with open ("trajectory.txt", "r") as file:
    for line in file:
        trajectory.append(ast.literal_eval(line.strip()))

servo_list=list()
servo_list.append(servo.Servo(pca.channels[0],min_pulse=700,max_pulse=2700))
servo_list.append(servo.Servo(pca.channels[1],min_pulse=600,max_pulse=2500))
servo_list.append(servo.Servo(pca.channels[2],min_pulse=600,max_pulse=2500))
servo_list.append(servo.Servo(pca.channels[3],min_pulse=400,max_pulse=2400))
servo_list.append(servo.Servo(pca.channels[4],min_pulse=450,max_pulse=2400))
servo_list.append(servo.Servo(pca.channels[5],min_pulse=450,max_pulse=2425))


for point in trajectory:
    servo_list[0].angle=90.0+point[0]*180.0/3.1415
    servo_list[1].angle=90.0+point[1]*180.0/3.1415
    servo_list[2].angle=90.0+point[2]*180.0/3.1415
    servo_list[3].angle=90.0+point[3]*180.0/3.1415
    servo_list[4].angle=90.0+point[4]*180.0/3.1415
    servo_list[5].angle=90.0+point[5]*180.0/3.1415
    time.sleep(1/F_POSITIONS)