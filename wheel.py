# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import logging
import multiprocessing as mp
import threading as th
import RPi.GPIO as gpio
from time import sleep
from gpiozero import Motor

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

    def __init__(self, pin):
        super().__init__()
        self.daemon = True
        self.pin = pin
        self._count = 0
        self.count_lock = th.Lock()

    def run(self):
        gpio.setup(self.pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        while True:
            sleep(0.01)
            gpio.wait_for_edge(self.pin, gpio.RISING)
            with self.count_lock:
                self._count += 1

    @property
    def count(self):
        with self.count_lock:
            return self._count


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
        self._speed = 0

    def forward(self, speed, clicks=None):
        self._parent_conn.send([COMMAND.FORWARD, speed, clicks])

    def backward(self, speed, clicks=None):
        self._parent_conn.send([COMMAND.BACKWARD, speed, clicks])

    def stop(self):
        self._parent_conn.send(COMMAND.STOP)

    @property
    def clicks(self):
        self._parent_conn.send(COMMAND.CLICKS)
        return self._parent_conn.recv()

    @property
    def speed(self):
        """positive value is forward, negative value is backward, zero is stopped"""
        self._parent_conn.send(COMMAND.SPEED)
        return self._parent_conn.recv()

    def run(self):
        self._monitor = WheelMonitor(self._monitor_pin)
        self._monitor.start()
        self._motor = Motor(self._forward_pin, self._backward_pin)
        self._motor.stop()
        while True:
            cmd = self._child_conn.recv()
            try:
                if cmd[0] == COMMAND.FORWARD:
                    self._forward(*self._speed_and_clicks(cmd))
                elif cmd[0] == COMMAND.BACKWARD:
                    self._backward(*self._speed_and_clicks(cmd))
                elif cmd[0] == COMMAND.STOP:
                    self._stop()
                elif cmd[0] == COMMAND.CLICKS:
                    self._child_conn.send(self._monitor.count)
                elif cmd[0] == COMMAND.SPEED:
                    self._child_conn.send(self._speed)
                else:
                    logger.warning('[%s] Ignoring invalid command: %s', self.name, cmd)
            except:
                logger.exception('[%s] Error handling command from parent: %s', self.name, cmd)

    def _forward(self, speed, clicks=None):
        if self._speed < 0:
            self._stop()
        self._motor.forward(speed)
        self._speed = speed
        # TODO handle clicks

    def _backward(self, speed, clicks=None):
        if self._speed > 0:
            self._stop()
        self._motor.backward(speed)
        self._speed = -1 * speed
        #TODO handle clicks

    def _stop(self):
        self._motor.stop()
        self._speed = 0

    @staticmethod
    def _speed_and_clicks(cmd):
        speed = cmd[1]
        if speed <= 0.0 or speed > 1.0:
            raise ValueError('Invalid speed value: %s' % speed)
        clicks = cmd[2] if len(cmd) > 2 else None
        if clicks and clicks < 1:
            raise ValueError('Invalid clicks value: %s' % clicks)
        return speed, clicks
