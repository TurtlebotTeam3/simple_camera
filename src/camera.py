#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')

        self.blob_detected = False

        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/image_raw', sensor_msgs.msg.Image, self.run)

        rospy.spin() 

    def run(self, image):
        #get frame from robo
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        #length ane width from frame
        numrows = len(frame)
        numcols = len(frame[0])
        #filter image and detect blob
        filteredImage = self.filterImage(frame)
        self.blob_detected = self.detectBlob(filteredImage, numrows, numcols, [-1] )

        #print and show image
        print "blob_detected: " + str(self.blob_detected) 
        cv2.imshow('img', filteredImage) 
        cv2.waitKey(5)

    def filterImage(self, frame):
        #convert to hsv image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
         # lower mask red
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        # upper mask red
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # join my masks for upper and lower red
        mask = mask0 + mask1
        # Bitwise-Not mask and original image
        res = cv2.bitwise_not(mask)
        return res 

    def detectBlob(self, image, resolX, resolY, xCenter):
        newimage = image
        minBlobWidth = 40
        xStart = -1
        for y in range(resolY):
            blobwidth = 0
            for x in range(resolX):
                pixel = newimage[x, y]
                #print pixel
                if pixel == 0:  # black pixel: a box!
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


