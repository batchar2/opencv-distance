# -*- coding: utf-8 -*-
import sys
import threading
import signal
import math

import cv2
import numpy as np


class Point:
    def __init__(self, x=-1, y=-1, radius=-1):
        self.x = x
        self.y = y
        self.radius = radius
    def __str__(self):
        return "%d" % self.radius


class Distance:
    def __init__(self, k, axis):
        self.axis = axis
        self.distance = -1
        self.k = k
        self.coordiantes = {'X': -1, 'Y': -1, 'Z': -1}

    def __str__(self):
        return "%d" % self.distance

    def mathDistance(self, contours, width, height):
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
                    self.dist = (width_point*self.k)/(float(point_distance)/float(width))

                    x, y = int((points[0].x + points[1].x)/2), int((points[0].y + points[1].y)/2)
                    pointO = Point(width/2, height/2)
                    pointA = Point(x, y)
                    pointB = Point(pointO.x, pointA.y)

                    OB = math.sqrt((pointO.x-pointB.x)**2 + (pointO.y-pointB.y)**2)
                    AB = math.sqrt((pointA.x-pointB.x)**2 + (pointA.y-pointB.y)**2)

                    sign_ab, sign_ob = 1, 1
                    if pointO.y < pointA.y: sign_ab = -1
                    #if pointO.x < pointA.x: sign_ob = -1


                    AB_cm = (AB/(point_distance))*width_point*sign_ob    
                    OB_cm = (OB/(point_distance))*width_point*sign_ab

                    AO_cm = math.sqrt(self.dist**2 - AB_cm**2 )
                    self.coordiantes[self.axis] = round(AO_cm, 1)          
                    self.coordiantes['Z'] = round(OB_cm, 1)


class CameraReaderThread(threading.Thread):
    def __init__(self, cam_number, axis, k, lock):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = False
        self.cam_number = cam_number
        self.distance = Distance(k, axis)
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
            self.distance.mathDistance(contours, width, height)
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
        cr1 = CameraReaderThread(0, 'X', 1.1, self.lock_readers[0])
        cr2 = CameraReaderThread(1, 'Y', 1.3, self.lock_readers[1])
        self.cam_readers = [cr1, cr2]

    def run(self):
        global frames

        self.cam_readers[0].start()
        self.cam_readers[1].start()
        index_cam = 1
        while not self.is_stop:
            self.lock_readers[0].acquire()
            self.lock_readers[1].acquire()

            c = cv2.waitKey(10)

            if c != -1:
                if index_cam == 0:
                    index_cam = 1
                else:
                    index_cam = 0 
            elif frames[index_cam] is not None:
                coor1 = self.cam_readers[0].distance.coordiantes
                coor2 = self.cam_readers[1].distance.coordiantes

                X = int(coor1['X'])
                Z1 = int(coor1['Z'])

                Y = int(coor2['Y'])
                Z2 = int(coor2['Z'])

                Z = (Z1 + Z2)/2
                coords = "X=%s Y=%s Z=%s" % (round(X, 1), round(Y, 1), round(Z, 1))
                
                cv2.putText(frames[index_cam], coords, (5, 30), cv2.FONT_HERSHEY_PLAIN, 2.0,(255,0,255))
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
