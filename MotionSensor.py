# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import logging
from time import sleep
from smbus2 import SMBus
from math import sqrt, degrees, atan2

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


class AK8963(MotionSensor):

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


class MPU9250(MotionSensor):

    def __init__(self, bus_id=1, i2c_addr=0x68, whoami_reg=0x75, iam_val=0x71):
        super().__init__(bus_id, i2c_addr, whoami_reg, iam_val)
        self.gyro_regs = (0x43, 0x45, 0x47)
        self.accl_regs = (0x3b, 0x3d, 0x3f)
        self.temp_reg = 0x41
        self.pwr_mgt_reg = 0x6b
        self.config_reg = 0x1a
        self.sample_rate_div_reg = 0x19
        self.gyro_config_reg = 0x1b
        self.accl_config_reg = 0x1c
        self.accl_config2_reg = 0x1d
        self.i2c_bypass_reg = 0x37
        self.int_enable_reg = 0x38
        self.ak8963 = AK8963(bus_id)

    def open(self, bus=None):
        super().open(bus)

        # Configure power mode
        self._write_byte(self.pwr_mgt_reg, 0x00)  # Clear sleep mode bit (6), enable all sensors
        sleep(0.100)
        self._write_byte(self.pwr_mgt_reg, 0x01)  # Auto select clock source
        sleep(0.200)

        # Configure gyro and thermometer
        self._write_byte(self.config_reg, 0x03)
        self._write_byte(self.sample_rate_div_reg, 0x04)  # 200Hz sample rate

        # Set gyro to full-scale range
        c = self.bus.read_byte_data(self.i2c_addr, self.gyro_config_reg)
        c &= ~0x02   # clear Fchoice bits [1:0]
        c &= ~0x18   # clear AFS bits [4:3]
        c |= 0 << 3  # Set full scale range for the gyro where 0 = 250DPS
        self._write_byte(self.gyro_config_reg, c)

        # Set accelerometer full-scale range
        c = self.bus.read_byte_data(self.i2c_addr, self.accl_config_reg)
        c &= ~0x18   # clear AFS bits [4:3]
        c |= 0 << 3  # Set full scale range for the gyro where 0 = 2G
        self._write_byte(self.accl_config_reg, c)

        # Set accelerometer sample rate
        c = self.bus.read_byte_data(self.i2c_addr, self.accl_config2_reg)
        c &= ~0x0F   # Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c |= 0x03    # Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        self._write_byte(self.accl_config2_reg, c)

        # The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, but all these rates
        # are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting.

        # Configure interruts and bypass-enable
        self._write_byte(self.i2c_bypass_reg, 0x22)  # enable I2C bypass so the AK8963 can join the I2C bus
        self._write_byte(self.int_enable_reg, 0x01)  # enable data ready (bit 0) interrupt
        sleep(0.100)

        self.ak8963.open(self.bus)

    @property
    def gyroscope(self):
        return self.read_sensor(self.gyro_regs)

    @property
    def accelerometer(self):
        return self.read_sensor(self.accl_regs)

    @property
    def rotation(self):
        return self.read_rotation(self.accl_regs)

    @property
    def temperature(self):
        return self._read_word_2c(self.temp_reg)

    @property
    def magnetometer(self):
        return self.ak8963.magnetometer


class MPU6050(MotionSensor):

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
