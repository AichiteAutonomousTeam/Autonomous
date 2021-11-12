#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pigpio
import time

HC_SR04_trig = 5
HC_SR04_echo = 6

pi = pigpio.pi()
pi.set_mode(HC_SR04_trig, pigpio.OUTPUT)
pi.set_mode(HC_SR04_echo, pigpio.INPUT)

t_rise = 0
t_fall = 0


def cbf(_, level, tick):  # パルスを検出
    global t_rise, t_fall
    if level:  # 立ち上がり
        t_rise = tick
    else:  # 立ち下がり
        t_fall = tick
        if t_fall >= t_rise:
            time_passed = t_fall - t_rise
        else:
            time_passed = t_fall + (0xffffffff + 1 - t_rise)

        # meter to cm, microseconds to seconds, divide by 2
        distance = 340 * 100 * time_passed / 1000000 / 2
        print('{"tick":%10d, "time_us": %6d, "distance_cm": %.2f}' % (tick, time_passed, distance))


pi.callback(HC_SR04_echo, pigpio.EITHER_EDGE, cbf)
pi.gpio_trigger(HC_SR04_trig, 10, 1)  # Trig (10μs pulse)
try:
    while True:
        time.sleep(0.1)  # wait for echo signal for 100msec (enough..., I believe...)
except KeyboardInterrupt:
    pass

pi.stop()
