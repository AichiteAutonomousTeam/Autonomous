#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import pigpio

PIN_A = [20, 21]  # left, right
# PIN_B = [19, 16]

pi = pigpio.pi()
pi.set_mode(PIN_A[0], pigpio.INPUT)
pi.set_mode(PIN_A[1], pigpio.INPUT)

cnt = [0, 0]  # L, R
cnt_cycle = [0, 0]  # L, R


def get_rotation_A(*_):
    if cnt[0] <= 1800:
        cnt[0] += 1
    else:
        cnt[0] = 0
        cnt_cycle[0] += 1
    print "A: ", cnt[0], cnt_cycle[0]


def get_rotation_B(*_):
    if cnt[1] <= 1800:
        cnt[1] += 1
    else:
        cnt[1] = 0
        cnt_cycle[1] += 1
    print "B: ", cnt[1], cnt_cycle[1]


if __name__ == '__main__':
    try:
        pi.callback(PIN_A[0], pigpio.RISING_EDGE, get_rotation_A)  # L
        # pi.callback(PIN_A[1], pigpio.RISING_EDGE, get_rotation_B)  # R
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    pi.stop()
