import threading
import pigpio
import time

SonicPin = [5, 6]
SendPin = 14

dst_max = 200.
max_sec = dst_max / 34300 * 2

pi = pigpio.pi()

pi.set_mode(SonicPin[0], pigpio.OUTPUT)
pi.set_mode(SonicPin[1], pigpio.INPUT)
pi.set_mode(SendPin, pigpio.OUTPUT)
pi.write(SendPin, 0)


if __name__ == '__main__':
    try:
        print "Sonic Ready"
        while True:
            pi.write(SonicPin[0], 1)
            time.sleep(0.00001)
            pi.write(SonicPin[0], 0)
            StartTime = time.time()
            StopTime = time.time()

            while not pi.read(SonicPin[1]) and (time.time() - StartTime) <= max_sec:
                StartTime = time.time()
            while pi.read(SonicPin[1]) and (time.time() - StartTime) <= max_sec:
                StopTime = time.time()

            distance = ((StopTime - StartTime) * 34300) / 2
            distance = distance if distance > 0 else 0
            if distance < 100:
                print(True)
            else:
                print(False)
    except KeyboardInterrupt:
        pass
    pi.write(SonicPin[0], 0)
    pi.stop()
