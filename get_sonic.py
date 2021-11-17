import threading
import pigpio
import time

SonicPin = [5, 6]
SendPin = 14

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

            while not pi.read(SonicPin[1]):
                StartTime = time.time()
            while pi.read(SonicPin[1]) and time.time() - StartTime <= 1:
                StopTime = time.time()

            distance = ((StopTime - StartTime) * 34300) / 2
            print int(distance)
            if distance < 50:
                pi.write(SendPin, 1)
            else:
                pi.write(SendPin, 0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    pi.write(SendPin, 0)
    pi.stop()
