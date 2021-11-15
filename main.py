#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import threading
import time

import pigpio
import rospy
import spidev
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

ThPins = [22, 23]  # Main(Ph15), Sub(Ph16)
SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)
SonicPin = [5, 6]  # Trigger, Echo

FreqHT = 1000
SteeringFreq = 30000  # Hzを上げると音が聞きづらくなるが、熱を持つ

pi = pigpio.pi()
spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz
pi.set_mode(SonicPin[0], pigpio.OUTPUT)
pi.set_mode(SonicPin[1], pigpio.INPUT)
for p in range(2):
    pi.set_mode(ThPins[p], pigpio.OUTPUT)
    pi.set_mode(SteeringPin[p], pigpio.OUTPUT)
    pi.set_PWM_frequency(SteeringPin[p], FreqHT)
    pi.set_PWM_range(SteeringPin[p], 255)


def steering_ang(num):
    if (num > 1) or (num < 0):
        return -1

    command = 0xd | (num << 1)
    command <<= 3
    ret = spi.xfer2([command, 0, 0])
    return int((ret[0] & 0x3) << 8 | ret[1])


def duty_to_percent(duty):
    return int(duty * 1000000 / 100.)


class Sonic(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.distance = 2000
        self.kill = False

    def run(self):
        while not self.kill:
            pi.write(SonicPin[0], 1)
            time.sleep(0.00001)
            pi.write(SonicPin[0], 0)
            StartTime = time.time()
            StopTime = time.time()

            while not pi.read(SonicPin[1]):
                StartTime = time.time()
            while pi.read(SonicPin[1]):
                StopTime = time.time()

            TimeElapsed = StopTime - StartTime
            self.distance = (TimeElapsed * 34300) / 2


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
            if self.ref - 10 < steering_ang(0) < self.ref + 10:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(0))
            elif self.ref - 20 < steering_ang(0) < self.ref + 20:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(13))
            else:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(17))

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


class WhiteLine(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.kill = False
        self.subscriber = rospy.Subscriber('/detect_whiteline', Bool, self.__line)
        self.count_timer = [0, 0]
        self.detected = False
    
    def run(self):
        while not self.kill:
            if self.count_timer[1] >= 3:
                self.detected = True
            if time.time() - self.count_timer[1] > 5:
                 self.count_timer = [0, 0]
                 self.detected = False
    
    def __line(self, _):
        if self.count_timer:
            self.count_timer[1] = time.time()
        else:
            self.count_timer[0] += 1


class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ad = 430
        rospy.init_node('Car')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__twist)

    def __twist(self, raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        rospy.logdebug("Autoware > %s" % self.twist)
        radius = (twist["speed"] / twist["ang"]) if twist["ang"] else 1  # 回転半径
        if abs(radius) < 15:
            self.ad = 260 if radius > 0 else 600
        else:
            self.ad = 430  # 中心
        self.speed = twist["speed"]

    def getTwist(self):
        return self.speed, self.ad

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        self.subscriber.unregister()


if __name__ == '__main__':
    try:
        rospy.get_published_topics()  # ros masterが立っていることを確認
    except Exception:
        sys.exit()
    else:
        sn = Sonic()
        st = Steering()
        ac = Accelerator()
        wl = WhiteLine()
        sn.start()
        st.start()
        ac.start()
        wl.start()
        try:
            a = Autoware()
            while not rospy.is_shutdown():
                stats = a.getTwist()
                ac.status = (stats[0] > 0) if sn.distance > 300 else False
                st.ref = stats[1]
                print ac.status, st.ref, steering_ang(0), sn.distance, wl.detected
                rospy.sleep(0.01)
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            pass
        st.kill = True
        ac.kill = True
    terminate()
