#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import sys
import threading
import time

import pigpio
import rospy
import spidev
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from logging import getLogger, StreamHandler, INFO, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

ThPins = [22, 23]  # Main(Ph15), Sub(Ph16)
SteeringPin = [27, 18]  # DIR(Ph13), PWM(Ph12)
SonicPin = [5, 6]  # Trigger, Echo

FreqHT = 1000  # ステアのPWMの周波数設定
SteeringFreq = 30000  # Hzを上げると音が聞きづらくなるが、Cytronが熱を持つ

dst_max = 200.  # 測定可能距離
max_sec = dst_max / 34300 * 2  # 測定可能距離から求めた最高時間

# 初期化
pi = pigpio.pi()
pi.set_mode(SonicPin[0], pigpio.OUTPUT)
pi.set_mode(SonicPin[1], pigpio.INPUT)
for p in range(2):
    pi.set_mode(ThPins[p], pigpio.OUTPUT)
    pi.set_mode(SteeringPin[p], pigpio.OUTPUT)
    pi.set_PWM_frequency(SteeringPin[p], FreqHT)
    pi.set_PWM_range(SteeringPin[p], 255)

spi = spidev.SpiDev()
spi.open(0, 0)  # bus0, CE0
spi.max_speed_hz = 1000000  # 1MHz


def steering_ang(num):
    """
    現在のステアリング角度を取得

    :param int num: 取得するチャンネル
    :return int : 取得した値
    """
    if (num > 1) or (num < 0):
        return -1

    command = 0xd | (num << 1)
    command <<= 3
    ret = spi.xfer2([command, 0, 0])
    return int((ret[0] & 0x3) << 8 | ret[1])


def duty_to_percent(duty):
    """
    引数のデューティ比(%)を
    pigpioの仕様(hardware_PWMの関数を使えるように)に変換する
    http://abyz.me.uk/rpi/pigpio/python.html#hardware_PWM

    :param int duty: %
    :return int : 0 (0%) - 1000000[1M](100%)
    """
    return int(duty * 1000000 / 100.)


class Sonic(threading.Thread):
    """
    デーモンの説明: https://docs.python.org/ja/2.7/library/threading.html#thread-objects から抜粋
    > スレッドには "デーモンスレッド (daemon thread)" であるというフラグを立てられます。
    > このフラグには、残っているスレッドがデーモンスレッドだけになった時に Python プログラム全体を終了させるという意味があります。
    """

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.flag = False  # 前に人がいる(True), いない(False)
        self.kill = False  # プログラム終了フラグ

    # run の説明: https://docs.python.org/ja/2.7/library/threading.html#threading.Thread.run
    def run(self):
        cnt = 0
        while not self.kill:
            # 抜粋: https://hellobreak.net/raspberrypi-ultrasonic-sensor-pipigpio-0122/
            pi.write(SonicPin[0], 1)
            time.sleep(0.00001)
            pi.write(SonicPin[0], 0)

            start_time = time.time()
            stop_time = time.time()

            while not pi.read(SonicPin[1]):
                start_time = time.time()
            while pi.read(SonicPin[1]) and (time.time() - start_time) <= max_sec:  # 一定以上経ったら処理スキップ
                stop_time = time.time()

            distance = ((stop_time - start_time) * 34300) / 2  # 距離計算
            logger.debug('sonic: {}'.format(distance))
            distance = True if 0 < distance < 100 else False  # 距離が0 < 100cm以下であればTrue
            cnt = (cnt + 1) if distance else 0  # 一回反応しただけでは敏感なため、「3回Trueを返したら」にする
            if distance and cnt >= 3:
                self.flag = True
                logger.info('sonic: STOP')
                time.sleep(3)  # 3秒間停止
                logger.info('sonic: START')
                self.flag = False
            time.sleep(0.1)


class Accelerator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.status = False  # 前進(True), 停止(False)
        self.kill = False

    def run(self):
        while not self.kill:
            # メインとサブが入れ替わった場合notを逆にする
            if not self.status:  # 発進
                pi.set_PWM_dutycycle(ThPins[0], 5 * 2.55)  # 5% -> 約0.5v
                pi.set_PWM_dutycycle(ThPins[1], 70 * 2.55)  # 70% -> 約4.5v
            elif self.status:  # 停止
                pi.set_PWM_dutycycle(ThPins[0], 70 * 2.55)  # 70% -> 約4.5v
                pi.set_PWM_dutycycle(ThPins[1], 5 * 2.55)  # 5% -> 約0.5v
            time.sleep(0.01)


class Steering(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.ref = 430  # 目標舵角
        self.kill = False

    def run(self):
        while not self.kill:
            pi.write(SteeringPin[0], self.ref < steering_ang(0))  # 右左どちらに動かすか
            if self.ref - 10 < steering_ang(0) < self.ref + 10:  # 現在の舵角から±10のとき動かさない
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(0))
            elif self.ref - 20 < steering_ang(0) < self.ref + 20:  # 現在の舵角から±20のとき弱く動かす
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(13))
            else:  # 現在の舵角からそれより大きいとき強く動かす
                pi.hardware_PWM(SteeringPin[1], SteeringFreq, duty_to_percent(17))

            time.sleep(0.1)


# 終了時に実行(初期状態に戻す)
def terminate():
    # noinspection PyBroadException
    try:
        pi.set_mode(SonicPin[1], pigpio.OUTPUT)
        for num in range(2):
            pi.write(ThPins[num], 0)
            pi.write(SteeringPin[num], 0)
            pi.write(SonicPin[num], 0)
    except Exception:
        pass
    finally:
        logger.info('Terminated!')
        pi.stop()


class WhiteLine(threading.Thread):
    """
    1回目の白線検出時から5秒以内に3回検出できれば停止/5秒以上でリセット
    """

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.count_timer = [0, 0.]  # [回数, 時間]
        self.detected = False  # 検出したか
        self.kill = False

        self.subscriber = rospy.Subscriber('/detect_whiteline', Bool, self.__line)  # detect_whitelineをサブスクライブ
        rospy.on_shutdown(self.subscriber.unregister)

    def __line(self, _):  # detect_whitelineがTrueを返したとき
        self.detected = False

class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ad = 430  # 目標舵角設定

        self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__twist)  # twist_cmdをサブスクライブ
        rospy.on_shutdown(self.subscriber.unregister)  # 終了時実行

    def __twist(self, raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        radius = (twist["speed"] / twist["ang"]) if twist["ang"] else 1  # 回転半径
        if abs(radius) < 15:  # 回転半径か15以上でステアを動かす
            self.ad = 260 if radius > 0 else 600
        else:
            self.ad = 430  # 中心
        self.speed = twist["speed"]

    def getTwist(self):
#        logger.debug('twist: {}, {}'.format(self.speed, self.ad))
        return self.speed, self.ad


if __name__ == '__main__':
    try:
        rospy.get_published_topics()  # ros masterが立っていることを確認
    except Exception:
        logger.critical('ROSCORE not found')
        sys.exit()
    else:
        rospy.init_node('Car')  # , log_level=rospy.DEBUG
        sn, st, ac = Sonic(), Steering(), Accelerator()
        # wl = WhiteLine()
        sn.start(), st.start(), ac.start()
        # wl.start()
        try:
            a = Autoware()
            while not rospy.is_shutdown():  # ctrl+Cやエラーがでるまでループ
                stats = a.getTwist()  # twist_cmdで処理したデータを取得
                if not sn.flag:  # 超音波センサの検知なし
                    ac.status = (stats[0] > 0)  # 速度が0より上で発進
                else:
                    ac.status = False  # 超音波センサの検知で停止
                st.ref = stats[1]  # 目標舵角を設定
                #if wl.detected:  # 白線検知したら
                #    while joystick_sw:  # joystickのスイッチが押されるまで待機
                #        pass
                rospy.sleep(0.01)
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            pass
        sn.kill, st.kill, ac.kill = True, True, True
        # wl.kill = True
    terminate()
