#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge
from PIL import Image as I

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')

        #cameraNode parameters
        self.COLOR = np.array([110,50,50])
        self.minAreaSize = 600
        self.blob_x = 0
        self.blob_y = 0
        self.blob_detected = False

        #subscribe to camera image
        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', sensor_msgs.msg.CompressedImage, self.run)
        #publisher blob detected
        self.pub = rospy.Publisher('blob_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(2)
        #publish false on init
        self.pub.publish(False)
        rospy.spin() 
        
        
    def run(self, image):
        #get frame from robo
        frame = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8')
        mask = self.calculateMask(frame)

        (self.blob_x, self.blob_y) = self.find_center(mask, self.minAreaSize)

        #publish
        #if not rospy.is_shutdown() and self.blob_detected != tempBlopDetected:
        #    self.pub.publish(tempBlopDetected)
        #    self.blob_detected = tempBlopDetected

        #show images
        cv2.imshow('normal_image', mask) 
        cv2.circle(mask, (self.blob_x, self.blob_y), 20, self.COLOR, thickness=5, lineType=8, shift=0)
        cv2.waitKey(10)

    def calculateMask(self, frame):
        im = cv2.blur(frame, (3, 3))

        # convert to hsv image
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        # green bounderies
        lowerBound = np.array([50, 50, 10], np.uint8)
        upperBound = np.array([90, 255, 255], np.uint8)

        mask = cv2.inRange(hsv, lowerBound, upperBound)

        return mask 
    
    def find_center(self, color, minArea):
        contours, hierarchy = cv2.findContours(color, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # find biggest blob
            max_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    best_blob = contour
            if max_area > minArea:
                # find centroid
                M = cv2.moments(best_blob)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(im, (cx, cy), 3, 255, -1)
                #(xcenter, ycenter), (MA, ma), angle = cv2.fitEllipse(best_blob)
                print "maxarea: " + str(max_area)
                print "position: " + str((cx, cy))
                return int(cx), int(cy)
            else:
                print "too small"
                print "maxarea: " + str(max_area)
                return 0, 0
        else:
            print "not found!"
            return 0, 0


if __name__ == '__main__':
	Camera()


