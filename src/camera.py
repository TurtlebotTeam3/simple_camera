#!/usr/bin/env python
import rospy
from simple_camera.msg import Blob
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
        self.minAreaSize = 2000
        #blob parameters
        self.blob_x = 0
        self.blob_y = 0
        self.blob_in_front = False

        #subscribe to camera image
        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', sensor_msgs.msg.CompressedImage, self._run)
        #publisher blob detected
        self.pub_blob = rospy.Publisher("blob", Blob, queue_size=10)
        self.rate = rospy.Rate(20)
        #publish false on init 
        self.blob_msg = Blob()
        self.blob_msg.blob_detected = self.blob_in_front
        self.blob_msg.blob_x = self.blob_x
        self.blob_msg.blob_y = self.blob_y
        self.pub_blob.publish(self.blob_msg)

        rospy.spin() 
        
        
    def _run(self, image):
        #get frame from robo
        frame = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8') 
        #TODO DELETE
        #cv2.imwrite('04.jpg',frame)
        #raw_input("stop")

        #self._showImage("img2", frame, False)
        #cut the top of the frame to only see 45 cm
        frame = frame[0:720, 0:1280]
        #calculate hsv mask
        mask = self._calculateMask(frame)
        #detect blob if x y == 0 no blob detected
        (self.blob_x, self.blob_y) = self._find_center(mask, self.minAreaSize)
        #publish
        if not rospy.is_shutdown() and self.blob_x != 0 and self.blob_y != 0:
            self.blob_in_front = True
            self.blob_msg.blob_detected = self.blob_in_front
            self.blob_msg.blob_x = self.blob_x
            self.blob_msg.blob_y = self.blob_y
            self.pub_blob.publish(self.blob_msg)
        else:
            self.blob_in_front = False
            self.blob_msg.blob_detected = self.blob_in_front
            self.blob_msg.blob_x = self.blob_x
            self.blob_msg.blob_y = self.blob_y
            self.pub_blob.publish(self.blob_msg)
        #show image with centroid
        self._showImage("img1", mask, True)
        

    def _showImage(self, name, img, centroid):
        #show images
        if centroid == True:
            cv2.circle(img, (self.blob_x, self.blob_y), 20, self.COLOR, thickness=5, lineType=8, shift=0)
        cv2.imshow(name, img) 
        cv2.waitKey(1)

    def _calculateMask(self, frame):
        im = cv2.blur(frame, (3, 3))
        # convert to hsv image
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        # green bounderies
        lowerBound = np.array([50, 50, 10], np.uint8)
        upperBound = np.array([90, 255, 255], np.uint8)
        # mask in Range of lowerBlue and upperBlue
        mask = cv2.inRange(hsv, lowerBound, upperBound)
        return mask 
    
    def _find_center(self, color, minArea):
        _, contours, _ = cv2.findContours(color, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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
                #cv2.circle(im, (cx, cy), 3, 255, -1)
                #(xcenter, ycenter), (MA, ma), angle = cv2.fitEllipse(best_blob)
                print "maxarea: " + str(max_area)
                print "position: " + str((cx, cy))
                return int(cx), int(cy)
            else:
                print "too small"
                print "maxarea: " + str(max_area)
                return 0, 0
        else:
            #print "not found!"
            return 0, 0

if __name__ == '__main__':
    try:
        camera=Camera()
    except rospy.ROSInterruptException:
        pass


