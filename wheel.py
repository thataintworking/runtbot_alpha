# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import sys
import logging
import multiprocessing as mp
import threading as th
import RPi.GPIO as gpio
from time import sleep
from gpiozero import Motor
from signal import signal, SIGTERM

logger = logging.getLogger('testdrivemp')


class COMMAND:
    FORWARD  = 'FWD'
    BACKWARD = 'BWD'
    STOP     = 'STP'
    CLICKS   = 'CLK'
    SPEED    = 'SPD'


class WheelMonitor(th.Thread):
    """
    This is implemented as a thread so there can only be one per process
    """

    def __init__(self, pin, mp_counter, mp_lock):
        super().__init__()
        self.daemon = True
        self.pin = pin
        self._counter = mp_counter
        self._counter_lock = mp_lock

    def run(self):
        gpio.setup(self.pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        self._counter.value = 0
        while True:
            sleep(0.01)
            gpio.wait_for_edge(self.pin, gpio.RISING)
            with self._counter_lock:
                self._counter.value += 1

    @property
    def count(self):
        with self._counter_lock:
            return self._counter.value


class Wheel(mp.Process):
    """
    Each wheel is run as a separate process because you can't have more than one interrupt in a single process.
    The wheel object includes both driving and monitoring functionality.
    """

    def __init__(self, forward_pin, backward_pin, monitor_pin, adjustment=0, name=None):
        super().__init__()
        self.daemon = True
        self.adjustment = adjustment
        self.name = name

        # the following values should be considered private after the object is constructed
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin
        self._monitor_pin = monitor_pin
        self._parent_conn, self._child_conn = mp.Pipe()
        self._monitor = None  # the WheelMonitor thread will get created and started when the Wheel process is started
        self._motor = None    # the Motor object will get created when the Wheel process is started
        self._speed = mp.Value('f', 0.0)
        self._speed_lock = mp.RLock()
        self._counter = mp.Value('i', 0)
        self._counter_lock = mp.RLock()

    def forward(self, speed):
        self._parent_conn.send([COMMAND.FORWARD, speed])

    def backward(self, speed):
        self._parent_conn.send([COMMAND.BACKWARD, speed])

    def stop(self):
        self._parent_conn.send(COMMAND.STOP)

    @property
    def clicks(self):
        with self._counter_lock:
            return self._counter.value

    @property
    def speed(self):
        """positive value is forward, negative value is backward, zero is stopped"""
        with self._speed_lock:
            return self._speed.value

    def run(self):
        self._monitor = WheelMonitor(self._monitor_pin, self._counter, self._counter_lock)
        self._monitor.start()
        self._motor = Motor(self._forward_pin, self._backward_pin)
        self._stop()

        def at_exit(*args):
            self._stop()
            sys.exit(0)

        signal(SIGTERM, at_exit)

        while True:
            cmd = self._child_conn.recv()
            try:
                if cmd[0] == COMMAND.FORWARD:
                    self._forward(self._speed_param(cmd))
                elif cmd[0] == COMMAND.BACKWARD:
                    self._backward(self._speed_param(cmd))
                elif cmd[0] == COMMAND.STOP or cmd == COMMAND.STOP:
                    self._stop()
                else:
                    logger.warning('[%s] Ignoring invalid command: "%s"', self.name, cmd)
            except:
                logger.exception('[%s] Error handling command from parent: "%s"', self.name, cmd)

    def _forward(self, speed):
        with self._speed_lock:
            if self._speed.value < 0:
                self._stop()
            self._motor.forward(speed + self.adjustment)
            self._speed.value = speed

    def _backward(self, speed):
        with self._speed_lock:
            if self._speed.value > 0:
                self._stop()
            self._motor.backward(speed + self.adjustment)
            self._speed.value = -1 * speed

    def _stop(self):
        self._motor.stop()
        with self._speed_lock:
            self._speed.value = 0

    @staticmethod
    def _speed_param(cmd):
        speed = cmd[1]
        if speed <= 0.0 or speed > 1.0:
            raise ValueError('Invalid speed value: %s' % speed)
        return speed
