#!/usr/bin/env python3

import rospy
from sentinel_drone.msg import Geolocation

def Plot():
    rospy.init_node('geolocation')
    geolocation = rospy.Publisher('/geolocation', Geolocation, queue_size=1)
    rate = rospy.Rate(0.01)
    location = Geolocation()
    location.objectid = "obj"
    location.lat = -122.156208
    location.long = 37.433864

    while not rospy.is_shutdown():
        location.lat -= 0.00002
        location.long += 0.00002
        geolocation.publish(location)
        rate.sleep()


if __name__ == '__main__':
    Plot()
