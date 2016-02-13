#import needed libraries
import numpy as np
import cv2
from video import create_capture
from common import clock,draw_str
import itertools as it
import sys
import operator
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


#define functions
def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh

#define webcam detection function
def detect():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        ret = frame.array
        img = frame.array
        '''img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)'''
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        rects, rand = hog.detectMultiScale(gray, winStride=(8,8), padding=(32,32), scale=1.05)

        for x, y, width, height in rects:
            cv2.rectangle(img, (x, y), (x+width, y+height), (255,0,0), 2)

        found_filtered = [];
        found_faces_filtered = [];

        for ri, r in enumerate(rects):
                for qi, q in enumerate(rects):
                    if ri != qi and inside(r, q):
                        break
                else:
                    found_filtered.append(r)

        for wi, w in enumerate(faces):
                for ei, e in enumerate(faces):
                    if wi != ei and inside(w, e):
                        break
                else:
                    found_faces_filtered.append(w)

        print('%d (%d) found FULLBODY' % (len(found_filtered), len(rects)))
        print('%d (%d) found FACES' % (len(found_faces_filtered), len(faces)))

        if len(found_filtered) in fullBodyNumDic:
            fullBodyNumDic[len(found_filtered)] = fullBodyNumDic[len(found_filtered)] + 1
        else:
            ind = len(found_filtered)
            fullBodyNumDic[ind] = 0

        if len(found_faces_filtered) in facesNumDic:
            facesNumDic[len(found_faces_filtered)] = facesNumDic[len(found_faces_filtered)] + 1
        else:
            ind = len(found_faces_filtered)
            facesNumDic[ind] = 0

        cv2.imshow('Person and Face Recognition + Counting', img)
        rawCapture.truncate(0)
        if cv2.waitKey(20) == 27:
            break


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
cascade_fn="haarcascade_frontalface_default.xml"
cascade= cv2.CascadeClassifier(cascade_fn)
# allow the camera to warmup
time.sleep(0.1)


hog = cv2.HOGDescriptor()
hog.setSVMDetector( cv2.HOGDescriptor_getDefaultPeopleDetector() )


cascPathFace = 'haarcascades_frontalface_alt.xml'
faceCascade = cv2.CascadeClassifier(cascPathFace)

facesNumDic = {0:0}
fullBodyNumDic = {0:0}

detect()

facesNumSorted = sorted(facesNumDic.items(), key=operator.itemgetter(0))
fullBodyNumSorted = sorted(fullBodyNumDic.items(), key=operator.itemgetter(0))
facesNumSortedValue = sorted(facesNumDic.items(), key=operator.itemgetter(1))
fullBodyNumSortedValue = sorted(fullBodyNumDic.items(), key=operator.itemgetter(1))

mostFaces = facesNumSorted[-1][0]
mostFullBodies = fullBodyNumSorted[-1][0]

print('Most Faces: ' + str(mostFaces))
print('Most Full Bodies: ' + str(mostFullBodies))

print(facesNumSorted)
print(fullBodyNumSorted)

if mostFaces < mostFullBodies:
    ind = 0
    for item in fullBodyNumSorted:
        if item[0] == mostFaces:
            ind = fullBodyNumSorted.index(item)
            del fullBodyNumSorted[0:ind]
            break
    highest = 0
    highestTuple = ()
    for i in fullBodyNumSorted:
        if i[1] >= highest:
            highest = i[1]
            highestTuple = i
    print('Number of people = ' + str(highestTuple[0]))

if mostFaces > mostFullBodies:
    ind = 0
    for item in facesNumSorted:
        if item[0] == mostFullBodies:
            ind = facesNumSorted.index(item)
            del facesNumSorted[0:ind]
            break
    highest = 0
    highestTuple = ()
    for i in facesNumSorted:
        if i[1] >= highest:
            highest = i[1]
            highestTuple = i
    print('Number of people = ' + str(highestTuple[0]))

if mostFaces == mostFullBodies:
    print('Number of people = ' + str(mostFaces))

cam.release()

cv2.destroyAllWindows()
