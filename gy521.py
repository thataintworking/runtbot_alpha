# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import math
from smbus2 import SMBus

GYRO_X = 0x43
GYRO_Y = 0x45
GYRO_Z = 0x47
ACCL_X = 0x3b
ACCL_Y = 0x3d
ACCL_Z = 0x3f


class GY521:
    
    def __init__(self, bus_id=1, i2c_addr=0x68, pwr_mgt=0x6b, gyro_scale=131, accl_scale=16384.0):
        self.bus_id = bus_id
        self.i2c_addr = i2c_addr
        self.pwr_mgt = pwr_mgt
        self.gyro_scale = gyro_scale
        self.accl_scale = accl_scale
        self.bus = SMBus(bus_id)
        self.bus.write_byte_data(i2c_addr, pwr_mgt, 0)  # wake the 6050 up as it starts in sleep mode

    def close(self):
        self.bus.close()

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.i2c_addr, adr)
    
    def read_word(self, adr):
        high = self.bus.read_byte_data(self.i2c_addr, adr)
        low = self.bus.read_byte_data(self.i2c_addr, adr + 1)
        val = (high << 8) + low
        return val
    
    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    @staticmethod
    def dist(a, b):
        return math.sqrt((a * a) + (b * b))
    
    @property
    def gyro_x(self):
        return self.read_word_2c(GYRO_X)

    @property
    def gyro_y(self):
        return self.read_word_2c(GYRO_Y)

    @property
    def gyro_z(self):
        return self.read_word_2c(GYRO_Z)

    @property
    def accl_x(self):
        return self.read_word_2c(ACCL_X)

    @property
    def accl_y(self):
        return self.read_word_2c(ACCL_Y)

    @property
    def accl_z(self):
        return self.read_word_2c(ACCL_Z)

    @property
    def gyro_x_scaled(self):
        return self.gyro_x / self.gyro_scale

    @property
    def gyro_y_scaled(self):
        return self.gyro_y / self.gyro_scale

    @property
    def gyro_z_scaled(self):
        return self.gyro_z / self.gyro_scale

    @property
    def accl_x_scaled(self):
        return self.accl_x / self.accl_scale

    @property
    def accl_y_scaled(self):
        return self.accl_y / self.accl_scale

    @property
    def accl_z_scaled(self):
        return self.accl_z / self.accl_scale

    @property
    def x_rotation(self):
        return math.degrees(math.atan2(self.accl_y, self.dist(self.accl_x, self.accl_z)))

    @property
    def y_rotation(self):
        return -math.degrees(math.atan2(self.accl_x, self.dist(self.accl_y, self.accl_z)))
