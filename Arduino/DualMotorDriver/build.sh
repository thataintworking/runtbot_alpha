#!/usr/bin/env bash

sbpath=$(cd `dirname $0`/.. && pwd)

arduino \
    --$1 \
    --board arduino:avr:nano:cpu=atmega328 \
    --port /dev/ttyUSB0 \
    --pref sketchbook.path=$sbpath \
    DualMotorDriver.cpp
