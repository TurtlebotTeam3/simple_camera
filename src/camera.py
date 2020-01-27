#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge
from PIL import Image as I
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
import tf
from geometry_msgs.msg._Pose import Pose
from tag_manager.srv import CheckTagKnown
from tag_manager.srv import AddTag
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
import math
from std_msgs.msg._Bool import Bool
import time
from time import gmtime, strftime
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')

        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback)

        #cameraNode parameters
        self.COLOR = np.array([110,50,50])
        self.minAreaSize = 600
        #blob parameters
        self.blob_x = 0
        self.blob_y = 0
        self.blob_in_front = False
        self.call_service = False
        self.map_resolution = 0
        self.map_offset_x = 0
        self.map_offset_y = 0
        self.received_map = False
        #subscribe to camera image
        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', sensor_msgs.msg.CompressedImage, self._run)

        #service tag_manager
        print "--- wait for check_tag_known service"
        rospy.wait_for_service('check_tag_known')
        self.tag_manager_check = rospy.ServiceProxy('check_tag_known', CheckTagKnown)
        print "--- check_tag_known service ready"
        rospy.wait_for_service('add_tag')
        print "--- wait for add_tag service"
        self.tag_manager_add = rospy.ServiceProxy('add_tag', AddTag)
        print "--- add_tag service ready"

        #simpel odom
        self.pose = Pose()
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', Pose, self._update_pose)

        #stop move to goal
        self.stop_move_to_goal_publisher = rospy.Publisher('move_to_goal/pause_action', Bool, queue_size=1)

        #move_to_goal is paused
        self.move_to_goal_is_paused_subscriber = rospy.Subscriber('/move_to_goal/paused', Bool, self._saveTag)

        print "--- CAMERA READY ---"
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

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        self.received_map = True

    def _robot_angle(self):
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
		return yaw  
        
    def _run(self, image):
        if self.received_map == True:
            #get frame from robo
            frame = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8') 
            #Save Iamge for H matrix
            #cv2.imwrite('03-03.jpg',frame)
            #raw_input("stop")

            #frame = frame[0:720, 0:1280]
            #calculate hsv mask
            mask = self._calculateMask(frame)
            #detect blob if x y == 0 no blob detected
            (self.blob_x, self.blob_y) = self._find_center(mask, self.minAreaSize)
            #publish
            if not rospy.is_shutdown() and self.blob_x != 0 and self.blob_y != 0:             
                if self.call_service == False:
                    self.call_service = True
                    #------------------------------------
                    # MoveToGoal --> STOP
                    #------------------------------------
                    self.stop_move_to_goal_publisher.publish(True)
            else:
                self.call_service = False

            #show image with centroid
            self._showImage("img1", mask, True)

    def _saveTag(self, data):
        if data.data == True:
        #if data == True:
            print strftime("%Y-%m-%d %H:%M:%S", gmtime())
            time.sleep(5)
            print strftime("%Y-%m-%d %H:%M:%S", gmtime())
            x_gobal, y_global = self._calculateMapPosOfTag(self.blob_x, self.blob_y)

            #print "robo globalx: " + str(self.pose.position.x)
            #print "robot globaly: " + str(self.pose.position.y)
            #print "tag globalx: " + str(x_gobal)
            #print "tag globaly: " + str(y_global)
            #print "robo in map_ x: " + str(int(math.floor((self.pose.position.x - self.map_offset_x)/self.map_resolution)))
            #print "robo in map_y: " + str(int(math.floor((self.pose.position.y - self.map_offset_y)/self.map_resolution)))
            #print "tag in map_ x: " + str(int(math.floor((x_gobal - self.map_offset_x)/self.map_resolution)))
            #print "tag in map_y: " + str(int(math.floor((y_global - self.map_offset_y)/self.map_resolution)))

            x_in_map = int(math.floor((x_gobal - self.map_offset_x)/self.map_resolution))
            y_in_map = int(math.floor((y_global - self.map_offset_y)/self.map_resolution))

            check_service_response = self.tag_manager_check(x_in_map,y_in_map)
            if check_service_response.tagKnown.data == False:
                add_service_response = self.tag_manager_add(x_in_map,y_in_map)
                print add_service_response
                print "Added New Tag at x:" + str(x_in_map) + " y: " + str(y_in_map)
            else:
                print "Already in List"
        #------------------------------------
        # MoveToGoal --> START
        #------------------------------------
        self.stop_move_to_goal_publisher.publish(False)
        
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
    
    def _calculateMapPosOfTag(self, x, y):
        H = np.array([[0.230489985832561, 3.57275008588951e-05, -146.602459530058],
                      [0.00589738679860119, 0.120006465885370, 257.753591375280],
                      [2.75956230239940e-05 , 0.0172475696150036, 1.0]])

        result = H.dot(np.asarray([x, y, 1]))

        x = result[1] / result[2]
        y = result[0] / result[2]

        _, _, phi = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])

        T = np.asarray(
            [
                [np.cos(phi), -np.sin(phi), self.pose.position.x],
                [np.sin(phi), np.cos(phi), self.pose.position.y],
                [0, 0, 1]
            ]
        )

        x, y, _ = np.matmul(T, np.asarray([(x/100), (y/100), 1]))

        return [x, y]



if __name__ == '__main__':
    try:
        camera=Camera()
    except rospy.ROSInterruptException:
        pass


