# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from .MotionSensor import MotionSensor


class MPU6050(MotionSensor):
    """A 6DOF motion sensor. Has no magnetometer."""

    def __init__(self, bus_id=1, i2c_addr=0x68, pwr_mgt=0x6b):
        super().__init__(bus_id, i2c_addr)
        self.gyro_regs = (0x43, 0x45, 0x47)
        self.accl_regs = (0x3b, 0x3d, 0x3f)

    @property
    def gyroscope(self):
        return self.read_sensor(self.gyro_regs)

    @property
    def accelerometer(self):
        return self.read_sensor(self.accl_regs)

    @property
    def rotation(self):
        return self.read_rotation(self.accl_regs)

    def print_data(self):
        x, y, z = self.gyroscope
        print("gyro x:", x)
        print("gyro y:", y)
        print("gyro z:", z)
        x, y, z = self.accelerometer
        print("accel x:", x)
        print("accel y:", y)
        print("accel z:", z)
        x, y = self.rotation
        print("rotation x:", x)
        print("rotation y:", y)
