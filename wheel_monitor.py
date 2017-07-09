# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

from multiprocessing import Process, Value, Lock
from time import sleep
import RPi.GPIO as gpio


class WheelMonitor(Process):
    """
    This needs to be a Process instead of a Thread because, apparently, you can't create
    multiple interrupts in the same process.
    """
    def __init__(self, pin, name):
        super().__init__()
        self.daemon = True
        self.pin = pin
        self.name = name
        self._count = Value('i', 0)  # create an integer variable in shared memory
        self.count_lock = Lock()

    def run(self):
        gpio.setup(self.pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        while True:
            sleep(0.01)
            gpio.wait_for_edge(self.pin, gpio.RISING)
            with self.count_lock:
                self._count.value += 1
                print('Click', self._count.value, 'on', self.name)

    @property
    def count(self):
        with self.count_lock:
            return self._count.value


def main():
    gpio.setmode(gpio.BCM)
    rw = WheelMonitor(8, 'left')
    lw = WheelMonitor(7, 'right')
    rw.start()
    lw.start()
    rw.join()
    lw.join()


if __name__ == '__main__':
    main()
