import pigpio
import time

TRIG = 5
ECHO = 6

pi = pigpio.pi()

pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)

def get_distance():
    pi.write(TRIG, 1)
    time.sleep(0.00001)
    pi.write(TRIG, 0)

    StartTime = time.time()
    StopTime = time.time()

    while not pi.read(ECHO):
        StartTime = time.time()

    while pi.read(ECHO):
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    return distance

if __name__ == '__main__':
    try:
        while True:
            dist = get_distance()
            if dist < 1000:
                print ("Distance = {} cm").format(int(dist))
            time.sleep(0.2)

    except KeyboardInterrupt:
        pi.stop()
