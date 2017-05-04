# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from MotionSensor import MPU9250

mpu = MPU9250()
mpu.open()

print()

x, y, z = mpu.gyroscope
print("gyro x:", x)
print("gyro y:", y)
print("gyro z:", z)

print()

x, y, z = mpu.accelerometer
print("accel x:", x)
print("accel y:", y)
print("accel z:", z)

print()

x, y = mpu.rotation
print("rotation x:", x)
print("rotation y:", y)

print()

x, y, z = mpu.magnetometer
print("mag x:", x)
print("mag y:", y)
print("mag z:", z)

print()

mpu.close()
