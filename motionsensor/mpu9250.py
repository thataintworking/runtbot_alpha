# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from time import sleep

from .MotionSensor import MotionSensor


class MPU9250(MotionSensor):
    """A 9DOF motion sensor."""

    gyro_addr = 0x68
    accl_addr = 0x68
    magn_addr = 0x0c
    temp_addr = 0x68
    pwr_mgt_reg = 0x6b
    config_reg = 0x1a
    sample_rate_div_reg = 0x19
    gyro_config_reg = 0x1b
    accl_config_reg = 0x1c
    accl_config2_reg = 0x1d
    i2c_bypass_reg = 0x37
    int_enable_reg = 0x38
    asa_regs = (0x10, 0x11, 0x12)
    control_reg = 0x0a

    def __init__(self, bus=None):
        super().__init__(bus,
                         gyro=(self.gyro_addr, 0x43, 0x45, 0x47),
                         accl=(self.accl_addr, 0x3b, 0x3d, 0x3f),
                         magn=(self.magn_addr, 0x03, 0x05, 0x07),
                         temp=(self.temp_addr, 0x41))

        # Configure power mode
        self._write_byte(self.gyro_addr, self.pwr_mgt_reg, 0x00)  # Clear sleep mode bit (6), enable all sensors
        sleep(0.100)
        self._write_byte(self.gyro_addr, self.pwr_mgt_reg, 0x01)  # Auto select clock source
        sleep(0.200)

        # Configure gyro and thermometer
        self._write_byte(self.gyro_addr, self.config_reg, 0x03)
        self._write_byte(self.gyro_addr, self.sample_rate_div_reg, 0x04)  # 200Hz sample rate

        # Set gyro to full-scale range
        c = self._read_byte(self.gyro_addr, self.gyro_config_reg)
        c &= ~0x02   # clear Fchoice bits [1:0]
        c &= ~0x18   # clear AFS bits [4:3]
        c |= 0 << 3  # Set full scale range for the gyro where 0 = 250DPS
        self._write_byte(self.gyro_addr, self.gyro_config_reg, c)

        # Set accelerometer full-scale range
        c = self._read_byte(self.gyro_addr, self.accl_config_reg)
        c &= ~0x18   # clear AFS bits [4:3]
        c |= 0 << 3  # Set full scale range for the gyro where 0 = 2G
        self._write_byte(self.gyro_addr, self.accl_config_reg, c)

        # Set accelerometer sample rate
        c = self._read_byte(self.gyro_addr, self.accl_config2_reg)
        c &= ~0x0F   # Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c |= 0x03    # Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        self._write_byte(self.gyro_addr, self.accl_config2_reg, c)

        # The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, but all these rates
        # are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting.

        # Configure interruts and bypass-enable
        self._write_byte(self.gyro_addr, self.i2c_bypass_reg, 0x22)  # enable I2C bypass so the AK8963 can join the I2C bus
        self._write_byte(self.gyro_addr, self.int_enable_reg, 0x01)  # enable data ready (bit 0) interrupt
        sleep(0.100)

        self._write_byte(self.magn_addr, self.control_reg, 0x00)  # power down (to reset the chip)
        sleep(0.010)
        self._write_byte(self.magn_addr, self.control_reg, 0x0f)  # enter fuse ROM access mode
        sleep(0.010)
        calibration_data = (
            float(self._read_byte(self.magn_addr, self.asa_regs[0]) - 128) / 256.0 + 1.0,
            float(self._read_byte(self.magn_addr, self.asa_regs[1]) - 128) / 256.0 + 1.0,
            float(self._read_byte(self.magn_addr, self.asa_regs[2]) - 128) / 256.0 + 1.0)

        # power down (to get out of fuse ROM access mode)
        self._write_byte(self.magn_addr, self.control_reg, 0x00)
        sleep(0.010)

        # Set magnetometer data resolution and sample ODR
        self._write_byte(self.magn_addr, self.control_reg, 0x01 << 4 | 0x02)
        sleep(0.010)

    def print_data(self):
        print('MPU9250')
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
