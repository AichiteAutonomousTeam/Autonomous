#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import sys
import threading
import time

import pigpio
import rospy
import spidev
from sensor_msgs.msg import Joy

ThPins = [22, 23]  # Main(Ph15), Sub(Ph16)
SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)
ReceivePin = 15
FreqHT = 1000
SteeringFreq = 30000  # Hzを上げると音が聞きづらくなるが、熱を持つ

pi = pigpio.pi()
spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz
for p in range(2):
    pi.set_mode(ThPins[p], pigpio.OUTPUT)
    pi.set_mode(SteeringPin[p], pigpio.OUTPUT)
    pi.set_PWM_frequency(SteeringPin[p], FreqHT)
    pi.set_PWM_range(SteeringPin[p], 255)
pi.set_mode(ReceivePin, pigpio.INPUT)



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
        pi.set_PWM_dutycycle(ThPins[0], 5 * 2.55)  # 5% -> 約0.5v
        pi.set_PWM_dutycycle(ThPins[1], 70 * 2.55)  # 70% -> 約4.5v



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
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(13))
            else:
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(17))

            time.sleep(0.1)
        pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(0))


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


class ROS:
    def __init__(self):
        self.joy = [False, 430]
        rospy.init_node('joystick')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.subscriber = rospy.Subscriber('/joy', Joy, self.__callback)

    def __callback(self, raw):
        # axes [左x, 左y, 右x, 右y, +字x, +字y]
        # -> [左y, 右x]
        self.joy = [raw.axes[1] if raw.axes[1] > 0 else 0, int(((raw.axes[2] + 1) * (260 - 600) / 2) + 600)]  # joystickの値をad角度に変換

    def get_twist(self):
        return self.joy

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        self.subscriber.unregister()


if __name__ == '__main__':
    try:
        rospy.get_published_topics()  # ros masterが立っていることを確認
    except Exception:
        print "ROSCOREが見つかりません"
        sys.exit()
    else:
        pro = subprocess.Popen("rosrun joy joy_node", shell=True)
        st, ac = Steering(), Accelerator()
        st.start(), ac.start()
        try:
            r = ROS()
            print "running"
            while not rospy.is_shutdown():
                status = r.get_twist()
                ac.status = True if status[0] else False
                st.ref = status[1]
                # print steering_ang(0), ac.status, st.ref

                rospy.sleep(0.01)
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            pass
        st.kill, ac.kill = True, True
        pro.kill()
    terminate()
