# testdrive.py
# Author: Ron Smith, That Ain't Working, All Rights Reserved
# This is just a simple script to see how the robot drives

from gpiozero import Motor
from time import sleep


def main():
    left_motor = Motor(17, 27)
    right_motor = Motor(23, 24)

    # move forward for 3 seconds
    left_motor.forward(0.5)
    right_motor.forward(0.5)
    sleep(3)
    left_motor.stop()
    right_motor.stop()

    # move backward for 3 seconds
    left_motor.backward(0.5)
    right_motor.backward(0.5)
    sleep(3)
    left_motor.stop()
    right_motor.stop()

    # turn left for 3 seconds
    left_motor.backward(0.5)
    right_motor.forward(0.5)
    sleep(3)
    left_motor.stop()
    right_motor.stop()

    # turn right for 3 seconds
    left_motor.forward(0.5)
    right_motor.backward(0.5)
    sleep(3)
    left_motor.stop()
    right_motor.stop()


if __name__ == '__main__':
    main()
