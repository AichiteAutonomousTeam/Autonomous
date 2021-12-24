#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
import sys
import time

import pigpio
import spidev

SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)

SteeringFreq = 20000  # Hzを上げると音が聞きづらくなるが、熱を持つ

pi = pigpio.pi()
spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz

for pin in range(2):
    pi.set_mode(SteeringPin[pin], pigpio.OUTPUT)

def duty_to_percent(duty):
    return int(duty * 1000000 / 100.)


def terminate():
    try:
        for pin in range(2):
            pi.write(SteeringPin[pin], 0)
    except Exception:
        pass
    finally:
        print("Terminated!")
        pi.stop()

def steering_ang(num):
    if (num > 1) or (num < 0):
        return -1

    command = 0xd | (num << 1)
    command <<= 3
    ret = spi.xfer2([command, 0, 0])
    return int((ret[0] & 0x3) << 8 | ret[1])


class Steering(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.ref = 474
        self.kill = False

    def run(self):
        while not self.kill:
            print steering_ang(0), self.ref < steering_ang(0), self.ref
            pi.write(SteeringPin[0], self.ref < steering_ang(0))
            if self.ref - 10 < steering_ang(0) < self.ref + 10:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(0))
            else:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(45))
            time.sleep(0.01)


if __name__ == '__main__':
    ste = Steering()
    ste.start()
    try:
        while True:
            ste.ref = 474
            time.sleep(6)
            ste.ref = 318
            time.sleep(6)
            ste.ref = 474
            time.sleep(6)
            ste.ref = 632
            time.sleep(6)
    except KeyboardInterrupt:
        pass
    terminate()
