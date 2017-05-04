# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from time import sleep
from MotionSensor import MPU9250

mpu = MPU9250()
mpu.open()

print("gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z, rot_x, rot_y, mag_x, mag_y, mag_z")
while True:
    print("%d, %d, %d, %d, %d, %d, %d, %d" % mpu.gyroscope + mpu.accelerometer + mpu.rotation + mpu.magnetometer)
    sleep(1)

mpu.close()
