# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import logging
from math import sqrt, degrees, atan2

from smbus2 import SMBus

logger = logging.getLogger(__name__)


class MotionSensor:
    """
    Base class for a motion sensor.
    """

    def __init__(self, bus_id, i2c_addr, whoami_reg, iam_val):
        self.bus_id = bus_id
        self.i2c_addr = i2c_addr
        self.whoami_reg = whoami_reg
        self.iam_val = iam_val
        self.bus = None
        self.ext_bus = False

    def __enter__(self):
        self.open(self.bus)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def open(self, bus=None):
        if self.bus:
            logger.warning("SMBus is already open.")
            return
        if bus:
            self.bus = bus
            self.ext_bus = True
        else:
            self.bus = SMBus(self.bus_id)
            self.ext_bus = False

    def close(self):
        if self.bus and not self.ext_bus:
            try:
                self.bus.close()
            except:
                logger.exception('Error closing SMBus.')
        self.bus = None

    def _write_byte(self, reg, value):
        self.bus.write_byte_data(self.i2c_addr, reg, value)

    def _read_byte(self, reg):
        return self.bus.read_byte_data(self.i2c_addr, reg)

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.i2c_addr, reg)
        low = self.bus.read_byte_data(self.i2c_addr, reg + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, reg):
        val = self._read_word(reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def whoami_check(self):
        return self._read_byte(self.whoami_reg) != self.iam_val

    # noinspection PyMethodMayBeStatic
    def dist(self, a, b):
        return sqrt((a * a) + (b * b))

    def calc_rotation(self, data):
        x, y, z = data
        return degrees(atan2(y, self.dist(x, z))), \
               -degrees(atan2(x, self.dist(y, z)))
        # TODO: calc z rotation when you figure out how

    def read_sensor(self, regs):
        x, y, z = regs
        return self._read_word_2c(x), \
               self._read_word_2c(y), \
               self._read_word_2c(z)

    def read_rotation(self, regs):
        return self.calc_rotation(self.read_sensor(regs))

