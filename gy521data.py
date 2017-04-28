# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from gy521 import GY521

gy = GY521()

print("gyro data")
print("---------")

print("gyro_xout: ", gy.gyro_x, " scaled: ", gy.gyro_x_scaled)
print("gyro_yout: ", gy.gyro_y, " scaled: ", gy.gyro_y_scaled)
print("gyro_zout: ", gy.gyro_z, " scaled: ", gy.gyro_z_scaled)

print()
print("accelerometer data")
print("------------------")

print("accel_xout: ", gy.accl_x, " scaled: ", gy.accl_x_scaled)
print("accel_yout: ", gy.accl_y, " scaled: ", gy.accl_y_scaled)
print("accel_zout: ", gy.accl_z, " scaled: ", gy.accl_z_scaled)

print("x rotation: ", gy.x_rotation)
print("y rotation: ", gy.y_rotation)

gy.close()
