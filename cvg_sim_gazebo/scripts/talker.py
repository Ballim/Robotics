#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


rospy.init_node('talker', anonymous=True)

empty=Empty()
twist=Twist()
twist.linear.x=1.0
twist.angular.z=0.0
stop=Twist()
stop.linear.x=0.0

takeoff=rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land=rospy.Publisher('ardrone/land', Empty, queue_size=1)
forward=rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

while takeoff.get_num_connections() < 1:
    rospy.loginfo_throttle(2, "Waiting for subscribers on /ardrone/takeoff ..")
    rospy.sleep(0.1)

    
rospy.sleep(2.0)
takeoff.publish(empty)
rospy.sleep(2.0)

now=rospy.Time.now()
rate=rospy.Rate(10)
while rospy.Time.now() < now + rospy.Duration.from_sec(3):
	forward.publish(twist)
	rate.sleep()
forward.publish(stop)
rospy.sleep(0.5)
land.publish(empty)
print("works")


