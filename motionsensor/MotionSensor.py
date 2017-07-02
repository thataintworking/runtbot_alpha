# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import logging

logger = logging.getLogger(__name__)


class MotionSensor:
    """
    Base class for a motion sensor.
    """

    def __init__(self, bus, gyro=None, accl=None, magn=None, temp=None):
        self.bus = bus
        self.gyro_ar = gyro
        self.accl_ar = accl
        self.magn_ar = magn
        self.temp_ar = temp

    def _write_byte(self, addr, reg, value):
        self.bus.write_byte_data(addr, reg, value)

    def _read_byte(self, addr, reg):
        return self.bus.read_byte_data(addr, reg)

    def _read_word(self, addr, reg):
        high = self.bus.read_byte_data(addr, reg)
        low = self.bus.read_byte_data(addr, reg + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, addr, reg):
        val = self._read_word(addr, reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def _read_sensor(self, ar):
        if not ar:
            return None
        addr, *regs = ar
        return tuple([self._read_word_2c(addr, r) for r in regs if r])

    @property
    def gyro(self):
        return self._read_sensor(self.gyro_ar)

    @property
    def accl(self):
        return self._read_sensor(self.accl_ar)

    @property
    def magn(self):
        return self._read_sensor(self.magn_ar)

    @property
    def temp(self):
        t = self._read_sensor(self.temp_ar)
        return t[0] if t else None
