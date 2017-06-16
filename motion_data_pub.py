# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import click
from time import sleep
from redis import StrictRedis
from motionsensor import MPU9250


@click.command()
@click.argument('channel', default='motion-sensor-data')
@click.option('--delay', '-d', default=0)
def main(channel, delay):
    with MPU9250() as s:
        r = StrictRedis()
        last_msg = ''
        while True:
            data = s.gyroscope + s.accelerometer + s.magnetometer + s.rotation + (s.temperature,)
            # noinspection PyStringFormat
            msg = '{"gx": %d, "gy": %d, "gz": %d, ' \
                  '"ax": %d, "ay": %d, "az": %d, ' \
                  '"mx": %d, "my": %d, "mz": %d, ' \
                  '"rx": %f, "ry": %f, "temp": %d}' \
                % data
            if msg != last_msg:
                print(msg)
                r.publish(channel, msg)
                last_msg = msg
            if delay:
                sleep(delay)


if __name__ == '__main__':
    main()
