#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import MapMetaData, OccupancyGrid, MapMetaData
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Empty, Int32MultiArray, Int16

def callback(data):
    print("callback")
    print(data)

def listener():
    while not rospy.is_shutdown():
        print("pre init")
        rospy.init_node('listener',anonymous=True)
        print("presub")
        rospy.Subscriber('map_data', numpy_msg(Int16), callback)
        rospy.sleep(10)
        # print("prespin")
        # rospy.spin()

if __name__=='__main__':
    listener() 