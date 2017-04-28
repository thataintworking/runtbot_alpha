# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from gy521 import GY521
from time import sleep

gy = GY521()

print("gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z, x_rotation, y_rotation")
while True:
    print("%d, %d, %d, %f, %f, %f, %f, %f" %
          (gy.gyro_x_scaled, gy.gyro_x_scaled, gy.gyro_z_scaled,
          gy.accl_x_scaled, gy.accl_y_scaled, gy.accl_z_scaled,
          gy.x_rotation, gy.y_rotation))
    sleep(1)
