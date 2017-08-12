# coding=utf-8
# Author: Ron Smith
# Copyright ©2017 That Ain't Working, All Rights Reserved

import RPi.GPIO as GPIO
from threading import Event
from subprocess import call

sd_event = Event()


def callback(pin):
    sd_event.set()


GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(16, GPIO.RISING, callback=callback)

sd_event.wait()

call('shutdown -h now "System shutdown initiated."', shell=True)


