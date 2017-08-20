#!/usr/bin/env bash

arduino \
    --$1 \
    --board arduino:avr:uno \
    --port /dev/ttyACM1 \
    EncoderInterruptExample.ino
