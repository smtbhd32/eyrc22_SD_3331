#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
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

#Importing for GDAL
from osgeo import gdal
import subprocess
import shlex


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control


		#Directory address of files.... Update it before running script... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		self.address = "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/"
		#Update it!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]
		self.block_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		#whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint =  [-9,-9.5,19]
		self.lastpoint = [0.0, 0.0, 19.0]


		#My Variables..........
		self.found = False
		self.rtn = False
		self.search = False
		self.id = -1
		self.ids = [-1]
		self.src = "drone.jpg"
		self.dest = "georeferenced.tif"
		self.tif = self.address + "task2d.tif"
		self.finalDest = "crs_updated.tif"
		self.csv_path = self.address + 'plot.csv'
		self.data = [['id', 'Longitude', 'Latitude']]
		

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [23.08,24.9,51.12]
		self.Ki = [0,0,0.8]
		self.Kd = [1218.3,1205.1,1400]



		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.error = [0.0,0.0,0.0]
		self.prev_error = [0.0,0.0,0.0]
		self.sum_error = [0.0,0.0,0.0]
		self.min = [1000,1000,1000]
		self.max = [2000,2000,2000]







		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		#self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		#Publishing Geolocation------------------------------------------------------------------------------------
		self.geolocation = rospy.Publisher('/geolocation', Geolocation, queue_size=1)

 



		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber("/edrone/camera_rgb/image_raw", Image, self.image_callback)




		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


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
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z




		
		#---------------------------------------------------------------------------------------------------------------





	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06
		self.Ki[1] = alt.Ki * 0.008
		self.Kd[1] = alt.Kd * 0.3

	def roll_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06
		self.Ki[0] = alt.Ki * 0.008
		self.Kd[0] = alt.Kd * 0.3
	
	#Image Callback function
	def image_callback(self,img_msg):

		# Try to convert the ROS Image message to a CV2 Image
		bridge = CvBridge()
		try:
			img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error:")

		if self.search == True:
			#640*480
			#Algo for checking if detected object is block, we're looking for:
			self.img = img.copy()
			if self.block_detect() == True:
				if self.found == False:
					self.lastpoint[0] = self.setpoint[0]
					self.lastpoint[1] = self.setpoint[1]
					self.setpoint[0] = self.drone_position[0]
					self.setpoint[1] = self.drone_position[1]
					print("HURRAY! Block Found !!!")
				self.found = True
		elif self.found == False:
			if self.id not in self.ids:
				self.Feature_Matching()
				self.geoloc()
				self.ids.append(self.id)

	#Algo for getting the location of yelloe block
	def block_detect(self):
		hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
		lower_ylow = np.array([90, 130, 130])
		upper_ylow = np.array([100, 255, 255])

		mask = cv2.inRange(hsv, lower_ylow, upper_ylow)
		corners = cv2.goodFeaturesToTrack(mask, 200, 0.001 , 5)
		if corners is None:
			return

		#Getting the corners of yellow object
		corners = np.int0(corners)
		detectedx = []
		detectedy = []
		for corner in corners:
			x, y = corner.ravel()
			detectedx.append(x)
			detectedy.append(y)
			
		#Detecting the x and y coordinates of yellow object
		x = int(sum(detectedx)/len(detectedx))
		y = int(sum(detectedy)/len(detectedy))
		Area = (max(detectedx)-min(detectedx))*(max(detectedy)-min(detectedy))

		if Area > 500 and x <= 500 and x >= 200 and y >= 150 and y <= 350:
			self.x = x
			self.y = y
			return True
		else:
			return False
			
		
			













	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
	
		self.error[2] = - (self.setpoint[2] - self.drone_position[2])
		self.error[1] = - (self.setpoint[1] - self.drone_position[1])
		self.error[0] =  (self.setpoint[0] - self.drone_position[0])

		self.cmd.rcThrottle = int(1500 + self.error[2] * self.Kp[2] + (self.error[2] - self.prev_error[2]) * self.Kd[2] + self.sum_error[2] * self.Ki[2])
		self.cmd.rcPitch = int(1500 + self.error[1] * self.Kp[1] + (self.error[1] - self.prev_error[1]) * self.Kd[1] + self.sum_error[1] * self.Ki[1])
		self.cmd.rcRoll = int(1500 + self.error[0] * self.Kp[0] + (self.error[0] - self.prev_error[0]) * self.Kd[0] + self.sum_error[0] * self.Ki[0])
		
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

		#Algo for checking if drone reached specified setpoint
		if abs(self.error[2]) <= 0.2 and abs(self.error[1]) <= 0.2 and abs(self.error[0]) <= 0.2:
			if self.found == True:
				if abs(self.x-320) >= 20:
					self.setpoint[0] += (self.x -320)/320
				if abs(self.y-240) >= 20:
					self.setpoint[1] += (self.y- 240)/300
				Errx = self.setpoint[0] - self.drone_position[0]
				Erry = self.setpoint[1] - self.drone_position[1]
				#Algo for turning off the camera when block is on the centre of the frame and making it move further...
				if abs(self.x-320) <= 10 and abs(self.y-240) <= 10:
					if abs(Errx) <= 0.2 and abs(Erry) <= 0.2:
						self.found = False
						self.setpoint[0] = self.lastpoint[0]
						self.setpoint[1] = self.lastpoint[1]
						self.search = False
						print("SEARCH OFF...")
						self.id += 1
						#Updating file names...
						self.src = "obj" + str(self.id)+ ".jpg"
						self.dest = "georeferenced_obj" + str(self.id) + ".tif"
						self.finalDest = "crs_updated_obj" + str(self.id) + ".tif"
						cv2.imwrite(self.src,self.img)
						self.move()
						print("MOVED AWAY...")
			else:
				if self.search == False:
					self.search = True
					print("TURNED SEARCH ON")
				self.move()
				

			

		



	#------------------------------------------------------------------------------------------------------------------------
		
		self.command_pub.publish(self.cmd)
		self.throttle_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[1])
		self.roll_error_pub.publish(self.error[0])
		

	#Publishing Geolocation
	def geoloc(self):
		#Creating location object for publishing long and lat
		if self.block_detect() == True:
			location = Geolocation()
			location.objectid = "obj" + str(self.id)
			#starting Gdal Library
			ds = gdal.Open(self.finalDest) # Open tif file
			# GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
			if ds is None:
				print("Could not open file!")
				return
			self.xoff, self.a, self.b, self.yoff, self.d, self.e = ds.GetGeoTransform()
			location.long, location.lat = self.pixel2coord(self.x, self.y)
			self.geolocation.publish(location)
			self.data.append([location.objectid,location.long,location.lat])
			print("Publishing on geolocation topic...")
			with open(self.csv_path, 'w', newline='') as csvfile:
				writer = csv.writer(csvfile)
				print("Writing data to csv file...")
				writer.writerows(self.data)

	#Search Algorithm
	def move(self):
		if self.rtn == False and self.found == False:
			if self.setpoint[1] >= 8 and self.setpoint[0] < 9: #Changing lane
				self.setpoint[0] += 4.5
				self.rtn = True
			elif self.setpoint[1] < 8:
				self.setpoint[1] += 2
			else:
				self.setpoint= [0,0,26]
		elif self.rtn == True and self.found == False:
			if self.setpoint[1] <= -8 and self.setpoint[0] < 9: #Changing lane
				self.setpoint[0] += 4.5
				self.rtn = False
			elif self.setpoint[1] > -8: 
				self.setpoint[1] -= 2.5
			else:
				self.setpoint= [0,0,26]

	#Getting Long Lat using GDAL Library
	def pixel2coord(self, x, y):
		"""Returns global pixel from pixel x, y coords"""
		xp = self.a * x + self.b * y + self.xoff
		yp = self.d * x + self.e * y + self.yoff
		return(xp, yp)  #longitude , #latitude

	#Algorithm for Feature Matching (SIFT) and Georeferencing using GDAL
	def Feature_Matching(self):
		#Detecting Features using SIFT Algo
		img1 = cv2.imread(self.src,0)
		img2 = cv2.imread(self.tif,0)

		#starting Gdal Library
		ds = gdal.Open(self.tif) # Open tif file
		# GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
		if ds is None:
			print("Could not open file!")
			return
		self.xoff, self.a, self.b, self.yoff, self.d, self.e = ds.GetGeoTransform()

		sift = cv2.SIFT_create(nfeatures = 500)

		kp1, des1 = sift.detectAndCompute(img1,None)
		kp2, des2 = sift.detectAndCompute(img2,None)

		# imgKp1 = cv2.drawKeypoints(img1,kp1,None)
		# imgKp2 = cv2.drawKeypoints(img2,kp2,None)

		bf = cv2.BFMatcher()
		matches = bf.knnMatch(des1, des2, k=2)

		good = []
		pixels = []

		for i,(m,n) in enumerate(matches):
			if m.distance < 0.50*n.distance:
				good.append([m])
				pixels.append(kp1[i].pt)
				pixels[len(pixels) - 1] = pixels[len(pixels) - 1] + kp2[i].pt
				# print(i,kp1[i].pt)
				# print(i,kp2[i].pt)
				# print(...)
		print("Detected Features...")
		#Georeferencing using Subprocess Module and GDAL
		command = "gdal_translate"
		for pixel in pixels:
			x,y = self.pixel2coord(pixel[2],pixel[3])
			# coord[len(coord) - 1] += pixel2coord(pixel[2],pixel[3])
			command = command + " -gcp " + str(pixel[0]) + " " + str(pixel[1]) + " " + str(x) + " " + str(y)

		command = command + " -of GTiff " + self.src + " " + self.dest

		subprocess.run(shlex.split(command))

		#Correcting the Coordinate System
		from_SRS = "EPSG:4326"
		to_SRS = "EPSG:4326"

		cmd_list = ["gdalwarp","-r", "bilinear", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", self.dest, self.finalDest]

		subprocess.run(cmd_list)

		




if __name__ == '__main__':
	time.sleep(6)
	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
