from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
from time import sleep
import cv2
import json

#fourcc = cv2.VideoWriter_fourcc(*'MP4V')
#kayit = cv2.VideoWriter('/home/pi/Desktop/kayit8.avi',fourcc,32.0,(640,480))
while True:
    with open('/home/pi/Desktop/Test/datagoruntu.json') as f:
        data = json.load(f)
    if data["camera"]:
        break
    sleep(0.1)

while True:
        
    prev_frame_time=0
    new_frame_time=0
    camera = PiCamera()
    camera.resolution =(640,480)
    camera.framerate =32
    rawCapture=PiRGBArray(camera,size=(640,480))
    
    sleep(0.1)
 
    for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        
        imgOriginal = frame.array 
        imgHSV=cv2.cvtColor(imgOriginal,cv2.COLOR_BGR2HSV)
        with open('/home/pi/Desktop/Test/datagoruntu.json') as f:
            data = json.load(f)
        
        if data["red"]:
            lower_blue = np.array([155, 110, 110])
            upper_blue = np.array([200, 255, 255])
        else :
            lower_blue = np.array([80,95,50])
            upper_blue = np.array([160,200,255])
        mask = cv2.inRange(imgHSV, lower_blue, upper_blue)
        result = cv2.bitwise_and(imgOriginal, imgOriginal, mask=mask)
        ( contours, hierarchy) = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        windowWidth=imgOriginal.shape[1]
        windowHeight=imgOriginal.shape[0]
        windowCenter=int(windowWidth/2), int(windowHeight/2)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if(area>75):
            
                x,y,w,h = cv2.boundingRect(contour)
                mask = cv2.rectangle(imgOriginal, (x,y), (x+w,y+h), (0, 0,255),2)
                
                
                if w > 50 and h > 40:
                    
                    recCenter= int((x+w)-(w/2)), int((y+h)-(h/2))
                    print(recCenter)
                    data = {
                        'x' : recCenter[0],
                        'y' : recCenter[1],
                        'tespit' : True
                        }
                    j = json.dumps(data)
                    #print(j)
                    with open("/home/pi/Desktop/Test/data.json","w") as f:
                        f.write(j)
                    sleep(0.1)
                    cv2.circle(imgOriginal, recCenter, 2 , (0, 0, 200), 4)
                    cv2.line(imgOriginal,windowCenter,recCenter, (200, 200, 200), 3)         
                    cv2.circle(imgOriginal, windowCenter, 1, (200, 0, 0), 1)
        #kayit.write(imgOriginal)
        #cv2.imshow("Frame",imgOriginal)
        #cv2.imshow("imgThresh",result)
 
        
        rawCapture.truncate(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    

kayit.release()
            
 




