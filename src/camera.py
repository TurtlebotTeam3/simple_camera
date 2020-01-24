#!/usr/bin/env python
import rospy
from simple_camera.msg import Blob
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge
from PIL import Image as I
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
import tf
from geometry_msgs.msg._Pose import Pose

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')

        #cameraNode parameters
        self.COLOR = np.array([110,50,50])
        self.minAreaSize = 200
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

        self.pose = Pose()
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', Pose, self._update_pose)

        rospy.spin() 

    def _update_pose(self, data):
		"""
		Update current pose of robot
		"""
		try:
			self.pose.position.x = data.position.x
			self.pose.position.y = data.position.y
			self.pose.position.z = data.position.z
			self.pose.orientation.x = data.orientation.x 
			self.pose.orientation.y = data.orientation.y 
			self.pose.orientation.z = data.orientation.z
			self.pose.orientation.w = data.orientation.w 
		except:
			print "ERROR --> TRANSFORM NOT READY"
		
		self.pose.position.x = round(data.position.x, 4)
		self.pose.position.y = round(data.position.y, 4)
		self.robot_yaw = self._robot_angle()   

    def _robot_angle(self):
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
		return yaw  
        
    def _run(self, image):
        #get frame from robo
        frame = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8') 
        #TODO DELETE
        #cv2.imwrite('03-03.jpg',frame)
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
            self._calculatePositionOfTag(self.blob_x, self.blob_y)
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
                #print "maxarea: " + str(max_area)
                #print "position: " + str((cx, cy))
                return int(cx), int(cy)
            else:
                #print "too small"
                #print "maxarea: " + str(max_area)
                return 0, 0
        else:
            #print "not found!"
            return 0, 0
    
    def _calculatePositionOfTag(self, x, y):
        H = np.array([[0.230489985832561, 3.57275008588951e-05, -146.602459530058],
                      [0.00589738679860119, 0.120006465885370, 257.753591375280],
                      [2.75956230239940e-05 , 0.0172475696150036, 1.0]])

        result = H.dot(np.asarray([x, y, 1]))

        x = result[1] / result[2]
        y = result[0] / result[2]

        _, _, phi = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])

        print phi
        print self.pose.position.x
        print self.pose.position.y
        print self.pose.orientation.z
        print self.pose.orientation.w

        T = np.asarray(
            [
                [np.cos(phi), -np.sin(phi), self.pose.position.x],
                [np.sin(phi), np.cos(phi), self.pose.position.y],
                [0, 0, 1]
            ]
        )

        x, y, z = np.matmul(T, np.asarray([(x/100), (y/100), 1]))

        print "tag at x: " + str(x)
        print "tag at y: " + str(y)
        print "tag at z: " + str(z)

        #pt = PointStamped()
        #pt.header.frame_id = "base_link"
        #pt.point.x = x
        #pt.point.y = y
        #pt.point.z = 1

        #self.marker_publisher.publish(pt)

        return [x, y]

if __name__ == '__main__':
    try:
        camera=Camera()
    except rospy.ROSInterruptException:
        pass


