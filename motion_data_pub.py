# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import click
from time import sleep
from redis import StrictRedis
from motionsensor import MPU9250, MinIMU9v5
from smbus2 import SMBusWrapper


NO_DATA = (None, None, None)


@click.command()
@click.argument('channel', default='motion-sensor-data')
@click.option('--delay', '-d', default=0)
def main(channel, delay):
    with SMBusWrapper(1) as bus:
        # s = MPU9250(bus)
        s = MinIMU9v5(bus)
        r = StrictRedis()
        last_msg = ''
        while True:
            data = (s.gyro or NO_DATA) + (s.accl or NO_DATA) + (s.magn or NO_DATA) + (s.temp,)
            # noinspection PyStringFormat
            msg = '{"gx": %s, "gy": %s, "gz": %s, ' \
                  '"ax": %s, "ay": %s, "az": %s, ' \
                  '"mx": %s, "my": %s, "mz": %s, ' \
                  '"temp": %s}' \
                % data
            if msg != last_msg:
                print(msg)
                r.publish(channel, msg)
                last_msg = msg
            if delay:
                sleep(delay)


if __name__ == '__main__':
    main()
