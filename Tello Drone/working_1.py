import cv2
import numpy as np
from djitellopy import tello
import time
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

me.takeoff()
time.sleep(1.5)
w, h = 360, 240 #86,400
fbRange = [1500, 2000]
fbRange_1 = [80000, 85000]

# pid = [0.7, 0.7, 0]
pid_x = [0.9, 0.9, 0]

pid = [1, 1, 0]
pError = 0
pError_x = 0

def findFace(img):
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 3)
    myFaceListC = []
    myFaceListArea = []
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        print(area)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
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
    if area < fbRange[0] and area != 0:
        fb = 50
    if area > fbRange[1] and area != 0:
        fb = 500

    if x == 0:
        speed = 0
        error = 0
    print(speed, fb)
    me.send_rc_control(0, fb, 0, speed)


    return error
 #attack face code   
# def atackFace( info, w, pid_x, pError_x):
#     area = info[1]
#     x, y = info[0]
#     fb = 0
#     error_x = x - w // 2
#     speed_x = pid_x[0] * error_x + pid_x[1] * (error_x - pError_x)
#     speed_x = int(np.clip(speed_x, -100, 100))
#     elif area < fbRange_1[0] and area != 0:
#         fb = 100
#     if x == 0:
#         speed = 0
#         error = 0
#     print(speed, fb)
#     me.send_rc_control(0, fb, 0, speed)
#     return error_x


#cap = cv2.VideoCapture(1)

while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))

    me.send_rc_control(0, 0, 0, 0)
    img, info = findFace(img)
    pError = trackFace(info, w, pid, pError)
    #pError_x = atackFace(info, w, pid_x, pError_x)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
