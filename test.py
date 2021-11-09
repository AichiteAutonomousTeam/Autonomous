#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
import time

import pigpio
import spidev

ThPins = [22, 23]  # Main(Ph15), Sub(Ph16)
SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)

FreqHT = 1000
SteeringFreq = 20000  # Hzを上げると音が聞きづらくなるが、熱を持つ

pi = pigpio.pi()
for pin in range(2):
    pi.set_mode(ThPins[pin], pigpio.OUTPUT)
    pi.set_mode(SteeringPin[pin], pigpio.OUTPUT)
    pi.set_PWM_frequency(SteeringPin[pin], FreqHT)
    pi.set_PWM_range(SteeringPin[pin], 255)

spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz


def terminate():
    try:
        pi.write(ThPins[0], 0)
        pi.write(ThPins[1], 0)
        pi.write(SteeringPin[0], 0)
        pi.write(SteeringPin[1], 0)
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


def duty_to_percent(duty):
    return int(duty * 1000000 / 100.)


# def accelerator(status):
#     if not status:
#         pi.set_PWM_dutycycle(ThPins[0], 5 * 2.55)  # 5% -> 約0.5v
#         pi.set_PWM_dutycycle(ThPins[1], 70 * 2.55)  # 70% -> 約4.5v
#     elif status:
#         pi.set_PWM_dutycycle(ThPins[0], 70 * 2.55)  # 70% -> 約4.5v
#         pi.set_PWM_dutycycle(ThPins[1], 5 * 2.55)  # 5% -> 約0.5v

class Accelerator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.status = False
        self.kill = False

    def run(self):
        while not self.kill:
            if not self.status:
                pi.set_PWM_dutycycle(ThPins[0], 5 * 2.55)  # 5% -> 約0.5v
                pi.set_PWM_dutycycle(ThPins[1], 70 * 2.55)  # 70% -> 約4.5v
            elif self.status:
                pi.set_PWM_dutycycle(ThPins[0], 70 * 2.55)  # 70% -> 約4.5v
                pi.set_PWM_dutycycle(ThPins[1], 5 * 2.55)  # 5% -> 約0.5v
            time.sleep(0.01)


class Steering(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.ref = 430
        self.kill = False

    def run(self):
        while not self.kill:
            pi.write(SteeringPin[0], self.ref < steering_ang(0))
            if self.ref - 5 < steering_ang(0) < self.ref + 5:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(0))
            elif self.ref - 10 < steering_ang(0) < self.ref + 10:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(20))
            else:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(40))

            time.sleep(0.1)


if __name__ == '__main__':
    # accelerator(False)
    ste = Steering()
    ac = Accelerator()
    ste.start()
    ac.start()
    try:
        i = 0
        while i < 500:
            print False
            ac.status = False
            i += 1
            time.sleep(0.01)
        while True:
            ang = 280
            print steering_ang(0), ang
            ste.ref = ang
            ac.status = True
            # accelerator(True)
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    ste.kill = True
    ac.kill = True
    terminate()
