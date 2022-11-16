from utils import *
import cv2
 
w,h = 360,240
pid = [0.6,0.4,0]
pErrorx = 0
pErrory = 0
pErrora = 0
startCounter = 0  # for no Flight 1   - for flight 0
 
 
myDrone = initializeTello()
 
while True:
 
    ## Flight
    if startCounter == 0:
        myDrone.takeoff()
        startCounter = 1
 
    ## Step 1
    img = telloGetFrame(myDrone,w,h)
    ## Step 2
    img, info = findFace(img)
    ## Step 3
    pErrorx,pErrory,pErrora = trackFace(myDrone,info,w,h,pid,pErrorx,pErrory,pErrora)
    #print(info[0][0])
    cv2.imshow('Image',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        myDrone.land()
        break
