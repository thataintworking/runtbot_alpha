# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved
from time import sleep

from .MotionSensor import MotionSensor


class AK8963(MotionSensor):
    """The AK8963 is a magnetometer only."""

    def __init__(self, bus_id, i2c_addr=0x0c, whoami_reg=0x00, iam_val=0x48):
        super().__init__(bus_id, i2c_addr, whoami_reg, iam_val)
        self.mag_regs = (0x03, 0x05, 0x07)
        self.asa_regs = (0x10, 0x11, 0x12)
        self.control_reg = 0x0a

    def open(self, bus=None):
        super().open(bus)
        self._write_byte(self.control_reg, 0x00)  # power down (to reset the chip)
        sleep(0.010)
        self._write_byte(self.control_reg, 0x0f)  # enter fuse ROM access mode
        sleep(0.010)
        calibration_data = (
            float(self._read_byte(self.asa_regs[0]) - 128) / 256.0 + 1.0,
            float(self._read_byte(self.asa_regs[1]) - 128) / 256.0 + 1.0,
            float(self._read_byte(self.asa_regs[2]) - 128) / 256.0 + 1.0
        )
        self._write_byte(self.control_reg, 0x00)  # power down (to get out of fuse ROM access mode)
        sleep(0.010)
        self._write_byte(self.control_reg, 0x01 << 4 | 0x02)  # Set magnetometer data resolution and sample ODR
        sleep(0.010)

    @property
    def magnetometer(self):
        return self.read_sensor(self.mag_regs)