import cv2
import numpy as np
from djitellopy import tello
import time
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
#me.takeoff()
time.sleep(1.5)
w, h = 360, 240 #86,400
fbRange = [80000, 81000]
pid = [1, 1, 0]
pError = 0
def findFace(img):
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 2)
    myFaceListC = []
    myFaceListArea = []
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]

    else:
        return img, [[0, 0], 0]
def trackFace( info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    fb = 0
    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -500, 500))
    if area > fbRange[0] and area < fbRange[1]:
        fb = -10
    elif area > fbRange[1]:
        fb = 10 #speed
    elif area < fbRange[0] and area != 0:
        fb = 300
    if x == 0:
        speed = 0
        error = 0
    print(speed, fb)
    i=0
    if i != -1:
        time.sleep(0.005)
        me.send_rc_control(0, fb, 0, speed)


    return error

# #cap = cv2.VideoCapture(1)

while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))

    img, info = findFace(img)
    pError = trackFace(info, w, pid, pError)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
