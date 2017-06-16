# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved
from time import sleep

from .ak8963 import AK8963
from .MotionSensor import MotionSensor


class MPU9250(MotionSensor):
    """A 9DOF motion sensor. Based on the MPU6050 but with an embedded AK8963 magnetometer."""

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
        x, y, z = self.magnetometer
        print("mag x:", x)
        print("mag y:", y)
        print("mag z:", z)
