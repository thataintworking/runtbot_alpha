# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from .MotionSensor import MotionSensor


class MPU6050(MotionSensor):
    """A 6DOF motion sensor. Has no magnetometer."""

    def __init__(self, bus=None):
        super().__init__(bus,
                         gyro=(0x68, 0x43, 0x45, 0x47),
                         accl=(0x68, 0x3b, 0x3d, 0x3f))

    def print_data(self):
        print('MCU6050')
        x, y, z = self.gyro
        print('gyro x:', x)
        print('gyro y:', y)
        print('gyro z:', z)
        x, y, z = self.accl
        print('accel x:', x)
        print('accel y:', y)
        print('accel z:', z)
