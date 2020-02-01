#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np
import cv2
import cv_bridge
import tf
import math

from PIL import Image as I
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Bool

from tag_manager.srv import CheckTagKnown
from simple_camera.msg import Blob
from simple_camera.srv import EnableBlobDetection, EnableBlobDetectionResponse, EnableTagKnownCheck, EnableTagKnownCheckResponse
from simple_odom.msg import CustomPose, PoseConverted

class Camera(): 

    def __init__(self):
        rospy.init_node('CameraNode')
        rospy.loginfo('Cam test node started')

        self.COLOR = np.array([110,50,50])
        self.minAreaSize = 1600 #6500
        self.blob_x = 0
        self.blob_y = 0
        self.blob_in_front = False
        
        self.map_info = MapMetaData()
        
        self.bridge = cv_bridge.CvBridge()
        
        self.pose = Pose()
        self.pose_converted = PoseConverted()
        
        self.do_blob_detection = True
        self.do_tag_known_check = True

        print("--- publisher ---")
        # --- Publishers ---
        self.blob_publisher = rospy.Publisher("blob", Blob, queue_size=10)
        self.stop_move_to_goal_publisher = rospy.Publisher('move_to_goal/pause_action', Bool, queue_size=1)
        self.move_to_tag_publisher = rospy.Publisher('move_to_tag_start_driving', Bool, queue_size=10)

        print("--- subscriber ---")
        # --- Subscribers ---
        self.camera_subscriber = rospy.Subscriber('/raspicam_node/image/compressed', sensor_msgs.msg.CompressedImage, self._run)
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', CustomPose, self._handle_update_pose)
        self.move_to_goal_is_paused_subscriber = rospy.Subscriber('/move_to_goal/paused', Bool, self._move_to_tag)

        print("--- service wait ---")
        # --- Service wait ---
        print("1")
        rospy.wait_for_service('check_tag_known')

        print("--- services ---")
        self.tag_manager_check_service = rospy.ServiceProxy('check_tag_known', CheckTagKnown)

        #publish false on init 
        self.blob_msg = Blob()
        self.blob_msg.blob_detected = False
        self.blob_msg.blob_x = 0
        self.blob_msg.blob_y = 0
        self.blob_publisher.publish(self.blob_msg)

        print("--- init service ---")
        # --- init service ---
        self.enable_blob_detection_service = rospy.Service('enable_blob_detection_service', EnableBlobDetection, self._set_blob_detection)
        self.enable_tag_known_check_service = rospy.Service('enable_tag_known_check_service', EnableTagKnownCheck, self._set_tag_known_check)

        print "--- CAMERA READY ---"
        rospy.spin() 

    def _setup(self):
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_info = map.info

    def _set_blob_detection(self,data):
        """
        Enable or disable the Blob detection
        """
        self.do_blob_detection = data.enableBlobDetection.data
        return EnableBlobDetectionResponse()

    def _set_tag_known_check(self,data):
        """
        Enable or disable the Tag Known check
        """
        self.do_tag_known_check = data.enableTagKnownCheck.data
        return EnableTagKnownCheckResponse()

    def _handle_update_pose(self, data):
        """
        Update current pose of robot
        """
        try:
            self.pose_converted = data.pose_converted
            self.pose = data.pose
        except:
            print('transform not ready')
        
    def _run(self, image):
        if self.do_blob_detection == True:
            #get frame from robo
            frame = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='bgr8') 
            #Save Iamge for H matrix
            #cv2.imwrite('03-03.jpg',frame)
            #raw_input("stop")

            #self._show_image("img2", frame, False)
            #cut the top of the frame to only see 45 cm
            frame = frame[0:720, 0:1280]
            #calculate hsv mask
            mask = self._calculate_mask(frame)
            #detect blob if x y == 0 no blob detected
            (self.blob_x, self.blob_y) = self._find_center(mask, self.minAreaSize)
            #publish
            if not rospy.is_shutdown() and self.blob_x != 0 and self.blob_y != 0:
                self.blob_msg.blob_detected = True
                self.blob_msg.blob_x = self.blob_x
                self.blob_msg.blob_y = self.blob_y
                self.blob_publisher.publish(self.blob_msg)              
                if self.blob_in_front == False:
                    self.blob_in_front = True
                    #------------------------------------
                    # MoveToGoal --> STOP
                    #------------------------------------
                    self.stop_move_to_goal_publisher.publish(True)
            else:
                self.blob_in_front = False
                self.blob_msg.blob_detected = False
                self.blob_msg.blob_x = 0
                self.blob_msg.blob_y = 0
                self.blob_publisher.publish(self.blob_msg)
            #show image with centroid
            #self._show_image("img1", mask, True)
        else:
            print "map not running"

    def _move_to_tag(self, data):
        # wenn stop true
        if data.data == True:
            if self.do_tag_known_check == True:
                next_x = self.pose_converted.x + math.cos(self.pose_converted.yaw) * 0.20
                next_y = self.pose_converted.y + math.sin(self.pose_converted.yaw) * 0.20
                robo_x_in_map = int(math.floor((next_x - self.map_info.origin.position.x)/self.map_info.resolution))
                robo_y_in_map = int(math.floor((next_y - self.map_info.origin.position.y)/self.map_info.resolution))
                check_service_response = self.tag_manager_check_service(robo_x_in_map,robo_y_in_map)
                if check_service_response.tagKnown.data == False:
                    #------------------------------------
                    # send move to Tag
                    #------------------------------------
                    msg_str = Bool()
                    msg_str = True
                    self.move_to_tag_publisher.publish(msg_str)
                else:
                    self.stop_move_to_goal_publisher.publish(False) 
            else:
                msg_str = Bool()
                msg_str = True
                self.move_to_tag_publisher.publish(msg_str)
        else: 
            self.stop_move_to_goal_publisher.publish(False)
        
    def _show_image(self, name, img, centroid):
        #show images
        if centroid == True:
            cv2.circle(img, (self.blob_x, self.blob_y), 20, self.COLOR, thickness=5, lineType=8, shift=0)
        cv2.imshow(name, img) 
        cv2.waitKey(1)

    def _calculate_mask(self, frame):
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

if __name__ == '__main__':
    try:
        camera=Camera()
    except rospy.ROSInterruptException:
        pass


