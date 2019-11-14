#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')
        #cameraNode parameters
        self.blob_detected = False
        self.minBlobWidth = 120
        #subscribe to camera image
        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/image_raw', sensor_msgs.msg.Image, self.run)
        #publisher blob detected
        self.pub = rospy.Publisher('blob_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(2)
        #publish false on init
        self.pub.publish(False)
        rospy.spin() 
        

    def run(self, image):
        #get frame from robo
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        #filter image and detect blob
        filteredImage = self.filterImage(frame)
        tempBlopDetected = self.detectBlob(filteredImage, len(frame), len(frame[0]), [-1] )
        #print and show image
        #print "blob_detected: " + str(self.blob_detected) 
        #cv2.imshow('img', filteredImage) 
        #publish
        if not rospy.is_shutdown() and self.blob_detected != tempBlopDetected:
            self.pub.publish(tempBlopDetected)
            self.blob_detected = tempBlopDetected
        cv2.waitKey(10)

    def filterImage(self, frame):
        #convert to hsv image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
         #lower mask red
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        #upper mask red
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        #join my masks for upper and lower red
        mask = mask0 + mask1
        #bitwise-not mask and original image
        res = cv2.bitwise_not(mask)
        return res 

    def detectBlob(self, image, resolX, resolY, xCenter):
        newimage = image
        minBlobWidth = self.minBlobWidth
        xStart = -1
        for y in range(resolY):
            blobwidth = 0
            for x in range(resolX):
                pixel = newimage[x, y]
                #print pixel
                if pixel == 0:  
                    blobwidth += 1
                    if  blobwidth == 1:
                        xStart = x
                else:
                    if blobwidth >= minBlobWidth:
                        xCenter[0] = xStart + blobwidth/2
                        #print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                        return True
                    elif blobwidth > 0:
                        blobwidth = 0
            if blobwidth >= minBlobWidth:
                xCenter[0] = xStart + blobwidth/2
                #print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                return True
        return False

if __name__ == '__main__':
	Camera()


