#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, cv2, time,math,os
import random2 as random
import numpy as np
from std_msgs.msg import Int64, Float64
from tulpar.msg import camera

liste = []
for i in range(1,1000):
    i = str(i)
    liste.append(i+'.avi')
a = random.choice(liste)

def imagePublish():
    konum = camera ()
    pre_radius = 0
    image_pub = rospy.Publisher ('radius', Float64, queue_size=1)
    konum_pub = rospy.Publisher ('konum', camera, queue_size=1)
    rospy.init_node ('image_publisher', anonymous=True)
    rate = rospy.Rate (20)
    dispW = 1024
    dispH = 768
    merkezX = 512
    merkezY = 384
    flip = 2
    fourcc = cv2.VideoWriter_fourcc (*'XVID')
    out = cv2.VideoWriter (a, fourcc, 15.0, (1024, 768))

    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
             'format=NV12, framerate=21/1 ! nvvidconv flip-method=' + str (flip) + \
             ' ! video/x-raw, width=' + str (dispW) + ', height=' + str (dispH) + \
             ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    cap = cv2.VideoCapture (camSet)
    time.sleep (2.0)
    kernel = np.ones ((5, 5), np.float32) / 25

    if not cap.isOpened ():
        print ("error cam cannot open")
        exit ()

    while not rospy.is_shutdown ():

        ret, frame = cap.read ()
        width = int (frame.shape[1])
        height = int (frame.shape[0])
        dim = (width, height)
        frame = cv2.resize (frame, dim)
        hsv_frame = cv2.cvtColor (frame, cv2.COLOR_BGR2HSV)

        #cv2.rectangle (frame, (600, 480), (360, 240), (0, 255, 0), 3)
        #blue
        lower_blue = np.array([94, 80, 2])
        upper_blue = np.array([120,255,255])
        mask_blue = cv2.inRange(hsv_frame,lower_blue,upper_blue)
        mask_blue =  cv2.erode(mask_blue,kernel)
        contours_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        
        
        """frame = cv2.line(frame, (450,330), (470,330), (0,255,0), 4)
        frame = cv2.line(frame, (450,330), (450,350), (0,255,0), 4)
        frame = cv2.line(frame, (510,330), (510,350), (0,255,0), 4)
        frame = cv2.line(frame, (510,330), (490,330), (0,255,0), 4)
        frame = cv2.line(frame, (510,390), (510,370), (0,255,0), 4)
        frame = cv2.line(frame, (510,390), (490,390), (0,255,0), 4)
        frame = cv2.line(frame, (450,390), (450,370), (0,255,0), 4)
        frame = cv2.line(frame, (450,390), (470,390), (0,255,0), 4)"""

        """ frame = cv2.line(frame, (0,384), (1024,384), (0,255,0), 4)
        frame = cv2.line(frame, (256,0), (256,768), (0,255,0), 4)
        frame = cv2.line(frame, (512,0), (512,768), (0,255,0), 4)
        frame = cv2.line(frame, (768,0), (768,768), (0,255,0), 4)


        frame = cv2.line(frame, (512,374), (512,394), (0,0,255), 4) #center crosshair
        frame = cv2.line(frame, (502,384), (522,384), (0,0,255), 4)""" #center crosshair
        
        
        out.write (frame)
        if len (contours_blue) > 0:
            #blue
            c = max (contours_blue, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle (c)
            centerX = int (x)
            centerY = int (y)

            cv2.circle(frame, (x,y), radius, (0, 0, 255), 2)
            frame = cv2.line(frame, (x,y), (x+radius,y), (0,0,255), 4)

            #******************CIRCLE********************* 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  
            # Blur using 3 * 3 kernel.
            gray_blurred = cv2.blur(gray, (3, 3))
            
            # Apply Hough transform on the blurred image.
            detected_circles = cv2.HoughCircles(gray_blurred, 
                            cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
                        param2 = 30, minRadius = 60, maxRadius = 300)
            
            # Draw circles that are detected.
            if detected_circles is not None:
            
                # Convert the circle parameters a, b and r to integers.
                detected_circles = np.uint16(np.around(detected_circles))
            
                for pt in detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]
            
                    # Draw the circumference of the circle.
                    cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
            
                    # Draw a small circle (of radius 1) to show the center.
                    cv2.circle(frame, (a, b), 1, (0, 255, 0), 3)


                konum.bolge = int (10)
                if r >= 0:

                    frame = cv2.line(frame, ((int(x)), (int(y) + 10)), ((int(x)), (int(y) - 10)), (0, 255,0), 5)
                    frame = cv2.line(frame, ((int(x) - 10), (int(y))), ((int(x) + 10), (int(y))), (0, 255,0), 5)
                    out.write(frame)


                    centerX2 = int (a)
                    centerY2 = int (b)

                    konum.farkx = int (centerX2- merkezX)
                    konum.farky = int (centerY2 - merkezY)
                    r2 = int (math.sqrt((konum.farkx**2)+(konum.farky**2)))
                    konum.bolge = int (r2)

                    frame = cv2.putText(frame, 'dx = {} '.format(konum.farkx), (830, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    frame = cv2.putText(frame, 'dy = {} '.format(konum.farky), (830, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    frame = cv2.putText(frame, 'dist = {} '.format(r2), (830, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    print("black detected","dx=",konum.farkx,"dy=",konum.farky,"r=",konum.bolge)

                    cv2.imwrite("/home/tulpar/Desktop/aa.png", frame)
                    konum_pub.publish (konum)
                    rate.sleep ()

                    if pre_radius < r:

                        pre_radius = r
                        image_pub.publish (r)
                        
                        rate.sleep ()
                        print ("radius=", r)
            """cv2.imshow("frame", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break"""
       
    cap.release ()
    cv2.destroyAllWindows ()


if __name__ == '__main__':
    try:
        imagePublish ()
    except rospy.ROSInterruptException:
        pass
