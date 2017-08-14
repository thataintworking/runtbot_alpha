#!/usr/bin/env bash

arduino \
    --$1 \
    --board ATTinyCore:avr:attinyx5:LTO=disable,TimerClockSource=default,chip=85,clock=8internal,bod=disable \
    TestResetAsSelect.ino
