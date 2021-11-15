import threading


class Sonic(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.distance = 2000
        self.kill = False

    def run(self):
        while not self.kill:
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
            self.distance = (TimeElapsed * 34300) / 2

