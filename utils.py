from djitellopy import Tello
import cv2
import numpy as np
 
 
def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone. left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone
 
def telloGetFrame(myDrone, w= 360,h=240):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame,(w,h))
    return img
 
def findFace(img):
    faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray,1.1,6  )
 
    myFaceListC = []
    myFaceListArea = []
 
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
        cx = x + w//2
        cy = y + h//2
        area = w*h
        myFaceListArea.append(area)
        myFaceListC.append([cx,cy])
 
    if len(myFaceListArea) !=0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i],myFaceListArea[i]]
    else:
        return img,[[0,0],0]
 
def trackFace(myDrone,info,w,h,pid,pErrorx,pErrory,pErrora):
 
    ## PID
    fbRange = [6200, 6800]
    area = info [1]
    x,y = info [0]
    errorx = x - w//2

    despx = x - w/2
    errorx = despx / (w/2)
    errorx = errorx
    speedx = pid[0]*errorx + pid[1]*(errorx-pErrorx)
    speedx = int(100*errorx)
    print (speedx)
    despy = y - h/2
    errory = despy / (h/2)
    errory = errory 
    speedy = pid[0]*errory + pid[1]*(errory-pErrory)
    speedy = int(100*errory)
    print (speedy)
    errora = area / 40000
    if errora > 0.155:
        errora = (errora - 0.155)*1.5
    elif errora < 0.155:
        errora = (errora - 0.155)*6.45
    speeda = pid[0]*errora + pid[1]*(errora-pErrora)
    speeda = int(100* -speeda)      
    if x !=0 and y != 0 and area != 0:
        myDrone.yaw_velocity = speedx
        myDrone.up_down_velocity = -speedy
        myDrone.for_back_velocity = speeda
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        errorx = 0
        errory = 0
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return errorx, errory, errora
