#!env /usr/bin python
import rospy
import cv2
#i add
import cv2 as cv
#
import numpy as np
import message_filters
from sensor_msgs.msg import Image

class detection:
    def __init__(self):

                        def plant_detection(self, rgb, mask, upper, lower):
                            hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV) #Transform BGR value to HSV;
                            #threshold = [upper, lower]
                            mask_after_segment = cv2.inRange(hsv,upper, lower)
                            cv2.imshow("mask_after_segment", mask_after_segment)
                            
                            '''
                            upper = np.array( [110,255, 58])
                            lower = np.array([105, 0, 45])
                            mask_ground = cv2.inRange(hsv,lower, upper)
                            
                            cv2.imshow("before_operation_ground", mask_ground)
                            '''

                            #Dilate first
                            kernel2 = np.ones((3,3),np.uint8)#create convolution
                            mask_after_dilate = cv2.dilate(mask_after_segment, kernel2, iterations=2) #mask after dalite 
                            #Erode
                            kernel = np.ones((2,2),np.uint8)#create convolution
                            mask_after_erode = cv2.erode(mask_after_dilate, kernel, iterations=3) # mask after erode
                            # cv2.imshow("mask_after_erode", mask_after_erode)#blank part is crop