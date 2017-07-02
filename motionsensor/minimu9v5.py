# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from .MotionSensor import MotionSensor


class MinIMU9v5(MotionSensor):
    """A 9DOF motion sensor."""

    gyro_addr = 0x6b
    accl_addr = 0x6b
    magn_addr = 0x1e
    temp_addr = 0x6b

    def __init__(self, bus=None):
        super().__init__(bus,
                         gyro=(self.gyro_addr, 0x22, 0x24, 0x26),
                         accl=(self.accl_addr, 0x28, 0x2a, 0x2c),
                         magn=(self.magn_addr, 0x28, 0x2a, 0x2c),
                         temp=(self.temp_addr, 0x20))

        self._write_byte(self.gyro_addr, 0x10, 0b10100111)   # CTRL1_XL
        self._write_byte(self.gyro_addr, 0x11, 0x00)         # CTRL2_G
        self._write_byte(self.gyro_addr, 0x14, 0b01100100)   # CTRL5_C
        self._write_byte(self.gyro_addr, 0x15, 0b00100000)   # CTRL6_C
        self._write_byte(self.gyro_addr, 0x16, 0x44)         # CTRL7_G

    def print_data(self):
        print('MinIMU9v5')
        x, y, z = self.gyro
        print('gyro x:', x)
        print('gyro y:', y)
        print('gyro z:', z)
        x, y, z = self.accl
        print('accl x:', x)
        print('accl y:', y)
        print('accl z:', z)
        x, y, z = self.magn
        print('magn x:', x)
        print('magn y:', y)
        print('magn z:', z)
        print('temp  :', self.temp)
