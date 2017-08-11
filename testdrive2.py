# testdrive.py
# Author: Ron Smith, That Ain't Working, All Rights Reserved
# This is just a simple script to see how the robot drives

from wheel import Wheel
from time import sleep
import RPi.GPIO as gpio


def main():
    gpio.setmode(gpio.BCM)
    left_wheel = Wheel(17, 27, 8, name='left')
    right_wheel = Wheel(23, 24, 7, name='right')

    left_wheel.start()
    right_wheel.start()

    lc = left_wheel.clicks
    rc = right_wheel.clicks

    print('Move forward for 3 seconds.')
    left_wheel.forward(1.0)
    right_wheel.forward(1.0)
    sleep(3)
    left_wheel.stop()
    right_wheel.stop()

    print('Clicks: left', left_wheel.clicks - lc, 'right', right_wheel.clicks - rc)
    lc = left_wheel.clicks
    rc = right_wheel.clicks

    sleep(1)

    print('Move backward for 3 seconds.')
    left_wheel.backward(1.0)
    right_wheel.backward(1.0)
    sleep(3)
    left_wheel.stop()
    right_wheel.stop()

    print('Clicks: left', left_wheel.clicks - lc, 'right', right_wheel.clicks - rc)
    lc = left_wheel.clicks
    rc = right_wheel.clicks

    sleep(1)

    print('Turn left for 3 seconds.')
    left_wheel.backward(1.0)
    right_wheel.forward(1.0)
    sleep(3)
    left_wheel.stop()
    right_wheel.stop()

    print('Clicks: left', left_wheel.clicks - lc, 'right', right_wheel.clicks - rc)
    lc = left_wheel.clicks
    rc = right_wheel.clicks

    sleep(1)

    print('Turn right for 3 seconds.')
    left_wheel.forward(1.0)
    right_wheel.backward(1.0)
    sleep(3)
    left_wheel.stop()
    right_wheel.stop()

    print('Clicks: left', left_wheel.clicks - lc, 'right', right_wheel.clicks - rc)


if __name__ == '__main__':
    main()
