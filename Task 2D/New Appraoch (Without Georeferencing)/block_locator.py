#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.ylowblk_pixelcoord[0]_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import csv
from sensor_msgs.msg import Image
from sentinel_drone.msg import Geolocation


# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Importing for GDAL
from osgeo import gdal
import subprocess
import shlex


class Edrone():
    """docstring for Edrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # Directory for address of files.... Update it before running script... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.address = "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/Working/"
        # Update it!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]
        self.block_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.setpoint = [-9, -9.5, 19]
        self.lastpoint = [0.0, 0.0, 19.0]

        # My Variables..........
        self.found = False
        self.rtn = False
        self.search = False
        self.id = -1
        self.ids = [-1]
        self.img_storage = "drone.jpg"
        self.sentinel_drone_map = self.address + "task2d.tif"
        self.csv_path = self.address + 'plot.csv'
        self.data = [['id', 'Longitude', 'Latitude']]
        self.ylowblk_pixelcoord = [0.0, 0.0]
        self.ylowblk_pixelcoord_tif = [0.0, 0.0]

        # Tell if we are in the middle of the lane, we are loosing the restrictions if true
        self.mid = False 

        # Opening sentinal drone map as image
        self.map = cv2.imread(self.sentinel_drone_map, 1)

        # starting Gdal Library
        ds = gdal.Open(self.sentinel_drone_map)  # Open tif file
        # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up.
        if ds is None:
            print("Could not open file!")
            return
        self.xoff, self.a, self.b, self.yoff, self.d, self.e = ds.GetGeoTransform()

        # Declaring a cmd of message type edrone_msgs and initializing values
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [23.08, 24.9, 51.12]
        self.Ki = [0, 0, 0.8]
        self.Kd = [1218.3, 1205.1, 1400]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.min = [1000, 1000, 1000]
        self.max = [2000, 2000, 2000]

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        # self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        # You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        # self.sample_time = 0.060 # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        self.throttle_error_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        # Publishing Geolocation------------------------------------------------------------------------------------
        self.geolocation = rospy.Publisher(
            '/geolocation', Geolocation, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)

        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber("/edrone/camera_rgb/image_raw",
                         Image, self.image_callback)

        # ------------------------------------------------------------------------------------------------------------

        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.

    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude

    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the ratio/fraction value accordingly
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    def pitch_set_pid(self, alt):
        self.Kp[1] = alt.Kp * 0.06
        self.Ki[1] = alt.Ki * 0.008
        self.Kd[1] = alt.Kd * 0.3

    def roll_set_pid(self, alt):
        self.Kp[0] = alt.Kp * 0.06
        self.Ki[0] = alt.Ki * 0.008
        self.Kd[0] = alt.Kd * 0.3

    # Image Callback function
    def image_callback(self, img_msg):

        # Try to convert the ROS Image message to a CV2 Image
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error:")

        # Algo for checking if detected object is block, we're looking for:
        if self.search == True:
            if self.block_detect(img) == True:
                self.img = img.copy()
                self.search = False
                self.id += 1
                self.mid = False
                if self.id not in self.ids:
                    if (self.publish_block_location() == True):
                        print("HURRAY! Block Found !!!")
                        self.ids.append(self.id)
                        # Turning Search On
                        self.search = True
                        return
            
    # Algo for getting the location of yelloe block
    def block_detect(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_ylow = np.array([90, 130, 130])
        upper_ylow = np.array([100, 255, 255])

        mask = cv2.inRange(hsv, lower_ylow, upper_ylow)
        corners = cv2.goodFeaturesToTrack(mask, 200, 0.001, 5)
        if corners is None:
            return

        # Getting the corners of yellow object
        corners = np.int0(corners)
        detectedx = []
        detectedy = []
        for corner in corners:
            x, y = corner.ravel()
            detectedx.append(x)
            detectedy.append(y)

        # Detecting the x and y coordinates of yellow object
        x = int(sum(detectedx)/len(detectedx))
        y = int(sum(detectedy)/len(detectedy))
        Area = (max(detectedx)-min(detectedx))*(max(detectedy)-min(detectedy))

        if Area > 500 and x <= 500 and x >= 200 and y >= 150 and y <= 350:
            self.ylowblk_pixelcoord = [x, y]
            return True
        else:
            return False

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        # 2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        # 3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        # 4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        # 5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        # 6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        # self.cmd.rcPitch = self.max_values[1]
        # 7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        # 8. Add error_sum

        self.error[2] = - (self.setpoint[2] - self.drone_position[2])
        self.error[1] = - (self.setpoint[1] - self.drone_position[1])
        self.error[0] = (self.setpoint[0] - self.drone_position[0])

        self.cmd.rcThrottle = int(1500 + self.error[2] * self.Kp[2] + (
            self.error[2] - self.prev_error[2]) * self.Kd[2] + self.sum_error[2] * self.Ki[2])
        self.cmd.rcPitch = int(1500 + self.error[1] * self.Kp[1] + (
            self.error[1] - self.prev_error[1]) * self.Kd[1] + self.sum_error[1] * self.Ki[1])
        self.cmd.rcRoll = int(1500 + self.error[0] * self.Kp[0] + (
            self.error[0] - self.prev_error[0]) * self.Kd[0] + self.sum_error[0] * self.Ki[0])

        if self.cmd.rcThrottle > self.max[2]:
            self.cmd.rcThrottle = self.max[2]
        if self.cmd.rcThrottle < self.min[2]:
            self.cmd.rcThrottle = self.min[2]

        if self.cmd.rcPitch > self.max[1]:
            self.cmd.rcPitch = self.max[1]
        if self.cmd.rcPitch < self.min[1]:
            self.cmd.rcPitch = self.min[1]

        if self.cmd.rcRoll > self.max[0]:
            self.cmd.rcRoll = self.max[0]
        if self.cmd.rcRoll < self.min[0]:
            self.cmd.rcRoll = self.min[0]

        self.prev_error[2] = self.error[2]
        self.prev_error[1] = self.error[1]
        self.prev_error[0] = self.error[0]

        self.sum_error[2] = self.sum_error[2] + self.error[2]
        self.sum_error[1] = self.sum_error[1] + self.error[1]
        self.sum_error[0] = self.sum_error[0] + self.error[0]

        # Algo for checking if drone reached specified setpoint
        if abs(self.error[2]) <= 0.3 and abs(self.error[1]) <= 0.3 and abs(self.error[0]) <= 0.3:
            self.search = True
            self.move()
        elif self.mid == True and abs(self.error[2]) <= 1 and abs(self.error[1]) <= 1 and abs(self.error[0]) <= 1:
            self.move() # less restrictions in middle of the lane

    # ------------------------------------------------------------------------------------------------------------------------

        self.command_pub.publish(self.cmd)
        self.throttle_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])

    # Search Algorithm
    def move(self):
        if self.rtn == False:
            if self.setpoint[1] >= 8 and self.setpoint[0] < 9:  # Changing lane
                self.setpoint[0] += 4.5
                self.rtn = True
            elif self.setpoint[1] < 8:
                self.setpoint[1] += 10
                self.mid = True
                if self.setpoint[1] > 8:
                    self.setpoint[1] = 8.5
                    self.mid = False
            else:
                self.setpoint = [0, 0, 26]
        elif self.rtn == True:
            if self.setpoint[1] <= -8 and self.setpoint[0] < 9:  # Changing lane
                self.setpoint[0] += 4.5
                self.rtn = False
            elif self.setpoint[1] > -8:
                self.setpoint[1] -= 10
                self.mid = True
                if self.setpoint[1] < -8:
                    self.setpoint[1] = -8.5
                    self.mid = False
            else:
                self.setpoint = [0, 0, 26]
        
    # Getting Long Lat using GDAL Library
    def pixel2coord(self, x, y):
        """Returns global pixel from pixel x, y coords"""
        xp = self.a * x + self.b * y + self.xoff
        yp = self.d * x + self.e * y + self.yoff
        return (xp, yp)  # longitude , #latitude

    # Algorithm for Feature Matching (SIFT) and then publishing the location of the block
    def publish_block_location(self):
        # Detecting Features using SIFT Algo
        img1 = self.img
        img2 = self.map

        descriptor = cv2.SIFT_create(2000)

        kp1, des1 = descriptor.detectAndCompute(img1, None)
        kp2, des2 = descriptor.detectAndCompute(img2, None)

        # Match the features using FLANN
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Find the good matches
        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        # Extract the matching points
        src_pts = np.float32(
            [kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32(
            [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # Estimate the transformation matrix using RANSAC
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        # Use the transformation matrix to map points from img1 to img2
        dst = cv2.warpPerspective(img1, M, (img2.shape[1], img2.shape[0]))

        # Define the pixel of interest in the first image
        sample_pixel = self.ylowblk_pixelcoord

        # Use the transformation matrix to map the sample pixel to the sentinel_drone_map.tif
        transformed_pixel = cv2.perspectiveTransform(np.array([[sample_pixel]], dtype=np.float32), M)

        # The transformed_pixel is a 2x1x2 array, so we can extract the x and y coordinates
        # of the transformed pixel in the sentinel_drone_map.tif
        x = int(transformed_pixel[0][0][0])
        y = int(transformed_pixel[0][0][1])

        # Print the location of the transformed pixel in the sentinel_drone_map.tif
        print("Transformed pixel location in sentinel_drone_map.tif: ({}, {})".format(x, y))
        
        # Checking if the block is already detected
        if abs(x - self.ylowblk_pixelcoord_tif[0]) < 100 and abs(y - self.ylowblk_pixelcoord_tif[1]) < 100:
            self.id -= 1
            return False

        # Storing the transformed pixel coordinates
        self.ylowblk_pixelcoord_tif = [x, y]

        # Creating location object for publishing long and lat
        location = Geolocation()
        location.objectid = "obj" + str(self.id)

		# Printing id of the block
        print("Object ID: {}".format(location.objectid))

		# Getting long and lat
        location.long, location.lat = self.pixel2coord(self.ylowblk_pixelcoord_tif[0], self.ylowblk_pixelcoord_tif[1])

		# Print the longitude and latitude of the block
        print("Longitude: {}, Latitude: {}".format(location.long, location.lat))
            
		# Appending data to csv file
        self.data.append([location.objectid, location.long, location.lat])

		# Publishing data
        print("Publishing on geolocation topic...")
        self.geolocation.publish(location)

		# Writing data to csv file
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            print("Writing data to csv file...")
            writer.writerows(self.data)

        # Everything went well
        return True


if __name__ == '__main__':
    time.sleep(3)
    e_drone = Edrone()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
