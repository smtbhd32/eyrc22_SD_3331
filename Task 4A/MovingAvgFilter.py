#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose

class PoseArrayFilter:
    def __init__(self):
        self.window_size = 50
        self.poses = []
        rospy.init_node('pose_array_filter')
        rospy.Subscriber('/whycon/poses', PoseArray, self.pose_array_callback)
        self.filtered_pub = rospy.Publisher('/moving_avg_pose_array_filter', PoseArray, queue_size=10)

    def pose_array_callback(self, msg):
        self.poses.append(msg)
        if len(self.poses) > self.window_size:
            self.poses.pop(0)
        filtered_poses = self.apply_filter()
        self.filtered_pub.publish(filtered_poses)

    def apply_filter(self):
        filtered_poses = PoseArray()
        for i in range(len(self.poses[0].poses)):
            x = [pose.poses[i].position.x for pose in self.poses]
            y = [pose.poses[i].position.y for pose in self.poses]
            z = [pose.poses[i].position.z for pose in self.poses]
            filtered_pose = self.get_average_pose(x, y, z)
            filtered_poses.poses.append(filtered_pose)
        return filtered_poses

    def get_average_pose(self, x, y, z):
        average_x = sum(x) / len(x)
        average_y = sum(y) / len(y)
        average_z = sum(z) / len(z)
        filtered_pose = Pose()
        filtered_pose.position.x = average_x
        filtered_pose.position.y = average_y
        filtered_pose.position.z = average_z
        return filtered_pose

if __name__ == '__main__':
    try:
        PoseArrayFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
