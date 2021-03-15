#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 # Messages used in the node must be imported.
from geometry_msgs.msg import Pose


import sys
import cv2
import numpy as np


rospy.init_node("track_blob")

cap=cv2.VideoCapture(0)




pub = rospy.Publisher('follow_blob', Pose, queue_size=10)

target_pose=Pose() # declaring a message variable of type Int32


x_d=0.0
y_d=0.0

while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	Purple_lower=np.array([120,105,80],np.uint8)# Purple Saturation values
	Purple_upper=np.array([155,255,255],np.uint8)# Purple Saturation values

	Purple=cv2.inRange(hsv,Purple_lower,Purple_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	Purple=cv2.dilate(Purple,kernal)

	img=cv2.circle(img,(445,227),5,(0,255,0),-1)# cv2.circle(img,(260,68),5,(255,0,0),-1) 474,245

			
	#Tracking the Purple Color
	(_,contours,hierarchy)=cv2.findContours(Purple,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	if len(contours)>0:
		contour= max(contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			img=cv2.circle(img,((2*x+w)/2,(2*y+h)/2),5,(0,255,0),-1)
			img=cv2.line(img,(445,227),((2*x+w)/2,(2*y+h)/2),(0,0,255),2)
		
			
			y_d= (((2*y+h)/2)-227) * 0.06 
                        x_d= (((2*x+w)/2)-445) * 0.075 
			
			s= 'x='+ str(x_d)+'  '+ 'y='+str(y_d)
			
			cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2,cv2.LINE_AA)
			
		
	cv2.imshow("Mask",Purple)
	cv2.imshow("Color Tracking",img)
	if cv2.waitKey(1)== ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

