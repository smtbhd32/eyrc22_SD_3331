#!/usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import PoseArray, Pose

class DroneTracker:
    def __init__(self):
        # Define the Kalman filter
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.x = np.zeros(6)
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])
        self.kf.P *= 1000
        self.kf.R = np.diag([0.1, 0.1, 0.1])

        # Initialize the node
        rospy.init_node('drone_tracker')

        # Set up ROS publishers and subscribers
        self.pose_sub = rospy.Subscriber('/whycon/poses', PoseArray, self.pose_callback)
        self.state_pub = rospy.Publisher('/kalman_pose_array_filter', PoseArray, queue_size=10)

    def pose_callback(self, pose_array):
        # Extract the position data from the PoseArray
        pos_x = [pose.position.x for pose in pose_array.poses]
        pos_y = [pose.position.y for pose in pose_array.poses]
        pos_z = [pose.position.z for pose in pose_array.poses]

        # Get the current time in seconds
        t = rospy.Time.now().to_sec()

        # Define the measurement vector
        z = np.array([pos_x[-1], pos_y[-1], pos_z[-1]])

        # Update the Kalman filter
        self.kf.predict()
        self.kf.update(z)

        # Publish the estimated state as a new PoseArray message
        state_msg = PoseArray()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = pose_array.header.frame_id

        state_pose = Pose()
        state_pose.position.x = self.kf.x[0]
        state_pose.position.y = self.kf.x[1]
        state_pose.position.z = self.kf.x[2]

        state_msg.poses.append(state_pose)
        self.state_pub.publish(state_msg)

if __name__ == '__main__':
    try:
        DroneTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
