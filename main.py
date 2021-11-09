#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
import time

import pigpio
import rospy
import spidev
from geometry_msgs.msg import TwistStamped

ThPins = [22, 23]  # Main(Ph15), Sub(Ph16)
SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)

FreqHT = 1000
SteeringFreq = 20000  # Hzを上げると音が聞きづらくなるが、熱を持つ

pi = pigpio.pi()
spi = spidev.SpiDev()
for p in range(2):
    pi.set_mode(ThPins[p], pigpio.OUTPUT)
    pi.set_mode(SteeringPin[p], pigpio.OUTPUT)
    pi.set_PWM_frequency(SteeringPin[p], FreqHT)
    pi.set_PWM_range(SteeringPin[p], 255)

spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz


def steering_ang(num):
    if (num > 1) or (num < 0):
        return -1

    command = 0xd | (num << 1)
    command <<= 3
    ret = spi.xfer2([command, 0, 0])
    return int((ret[0] & 0x3) << 8 | ret[1])


def duty_to_percent(duty):
    return int(duty * 1000000 / 100.)


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


def terminate():
    # noinspection PyBroadException
    try:
        for num in range(2):
            pi.write(ThPins[num], 0)
            pi.write(SteeringPin[num], 0)
    except Exception:
        pass
    finally:
        print("Terminated!")
        pi.stop()


class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ad = 430
        rospy.init_node('Car')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__callback)
        # self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.__callback)

    def __callback(self, raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # self.twist = {"speed": raw.linear.x, "ang": raw.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        rospy.logdebug("Autoware > %s" % self.twist)
        radius = (twist["speed"] / twist["ang"]) if twist["ang"] else 1  # 回転半径
        if abs(radius) < 20:
            self.ad = 260 if radius > 0 else 600
        else:
            self.ad = 430  # 中心
        self.speed = twist["speed"]

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        self.subscriber.unregister()

    def getTwist(self):
        return self.speed, self.ad


if __name__ == '__main__':
    try:
        rospy.get_published_topics()  # ros masterが立っていることを確認
    except Exception:
        sys.exit()
    else:
        s = Steering()
        ac = Accelerator()
        s.start()
        ac.start()
        try:
            a = Autoware()
            while not rospy.is_shutdown():
                stats = a.getTwist()
                ac.status = stats[0] > 0
                s.ref = stats[1]
                rospy.sleep(0.01)
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            pass
        s.kill = True
        ac.kill = True
    terminate()
