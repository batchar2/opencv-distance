# -*- coding: utf-8 -*-
import sys
import threading
import signal

import cv2
import numpy as np


class Point:
    def __init__(self, x=-1, radius=-1):
        self.x = x
        self.radius = radius
    def __str__(self):
        return "%d" % self.radius

class Distance:
    def __init__(self, k):
        self.distance = -1
        self.k = k

    def __str__(self):
        return "%d" % self.distance

    def mathDistance(self, contours, width):
        i = 0
        points = []
        for cnt in contours:
            (x,y), radius = cv2.minEnclosingCircle(cnt)
            points.append(Point(x, radius))

        points.sort(key=lambda x: x.radius,  reverse=True)

        if len(points) >= 2:
            point_distance = abs(points[0].x - points[1].x)
            width_point = 5.0
            if point_distance != 0:
                self.distance = (width_point*self.k)/(float(point_distance)/float(width))

class CameraReaderThread(threading.Thread):
    def __init__(self, cam_number, k, lock):
        threading.Thread.__init__(self)
        self.daemon = False
        self.cam_number = cam_number
        self.distance = Distance(k)
        self.lock = lock
        self.killed = False

    def kill(self):
        self.killed = True

    def run(self):
        global frames

        camera = cv2.VideoCapture(self.cam_number)
        _, frame = camera.read()
        
        height, width, depth = frame.shape
        
        while not self.killed:
            

            _, frame = camera.read()
            frames[self.cam_number] = frame

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        
            ret, img = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.lock.acquire()
            self.distance.mathDistance(contours, width)
            self.lock.release()

class SignalHandler:
    def __init__(self, target):
        self.target=target

    def handle_sigterm(self, signum, frame):
        self.target.is_stop = True

class Program:
    def __init__(self):
        self.is_stop = False
        self.stoped_key = False

        sh = SignalHandler(self)
        signal.signal(signal.SIGTERM, sh.handle_sigterm)
        signal.signal(signal.SIGINT, sh.handle_sigterm)

        self.lock_readers = [threading.Lock(), threading.Lock()]

        self.cam_readers = [CameraReaderThread(0, 1.1, self.lock_readers[0]), CameraReaderThread(1, 1.3, self.lock_readers[1])]

    def run(self):
        global frames

        self.cam_readers[0].start()
        self.cam_readers[1].start()
        index_cam = 1
        while not self.is_stop:
            self.lock_readers[0].acquire()
            self.lock_readers[1].acquire()

            c = cv2.waitKey(10)

            if c == 1048603:
                self.is_stop = True
            elif c != -1:
                if index_cam == 0:
                    index_cam = 1
                else:
                    index_cam = 0 
            elif frames[index_cam] is not None:
                coords = "X=%s Y=%s" % (self.cam_readers[0].distance, self.cam_readers[1].distance)
                cv2.putText(frames[index_cam], coords, (5, 30), cv2.FONT_HERSHEY_PLAIN, 2.0,(0,0,255))
                cv2.imshow('img', frames[index_cam])

            self.lock_readers[1].release()
            self.lock_readers[0].release()

        self.stop()

    def stop(self):
        self.cam_readers[0].kill()
        self.cam_readers[1].kill()
        self.cam_readers[0].join()
        self.cam_readers[1].join()
        sys.exit()

if __name__ == '__main__':
    frames = [None, None]
    program = Program()
    program.run()
