#!/usr/bin/env python
#Group Members: Philani Mpofu and Jonathan Wu
import rospy
import sys
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from time import sleep

def flyTo(coordinates):
    rospy.init_node('flyTo', anonymous=True)
    rate = rospy.Rate(10)

    #publishers
    takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    land = rospy.Publisher('ardrone/land', Empty, queue_size=10)

    #check for a connection
    while ((takeoff.get_num_connections() < 1) or (land.get_num_connections() < 1)):
        sleep(0.5)

    #takeoff message
    takeoff.publish()
    sleep(2.5)
    rate.sleep()

    kI = 0; kD = 0; kP = 0.5;
    kIz = 0.3; kDz = 0; kPz = 0.5

    error_sum = [0,0,0]; angle_error_sum = 0;
    prev_error = [0,0,0]; prev_angle_error = 0;
    diff_error = [0,0,0]; angle_diff_error = 0;

    quadAngularDest = 0; quad_coordinates = [0,0,0]; dest_coordinates = np.array(coordinates); num_iter = 0;
    while((np.linalg.norm(dest_coordinates-quad_coordinates,2) > 0.075) or (num_iter == 0)):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        resp_coordinates = model_coordinates('quadrotor',"")
        print(resp_coordinates)
        quad_coordinates = np.array([resp_coordinates.pose.position.x,resp_coordinates.pose.position.y,resp_coordinates.pose.position.z])
        quadAngular = resp_coordinates.twist.angular.z

        print(quad_coordinates)
        print(dest_coordinates)

        angle_error = quadAngularDest - quadAngular
        error = dest_coordinates-quad_coordinates
        print(error)

        angle_error_sum += angle_error
        error_sum += error

        angle_diff_error = abs(angle_error - prev_angle_error)
        diff_error = abs(error - prev_error)
        prev_error = error
        angle_diff_error = angle_error

        U = kP*error + kI*error_sum + kD*diff_error
        UAngular = kPz*angle_error + kIz*angle_error_sum + kDz*angle_diff_error

        print(U[0])
        print(U[1])
        print(U[2])

        vel_msg = Twist()
        vel_msg.linear.x = U[0]
        vel_msg.linear.y = U[1]
        vel_msg.linear.z = U[2]
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = UAngular
        move.publish(vel_msg)
        rate.sleep()
        num_iter = num_iter + 1

    rate.sleep()

if __name__ == '__main__':
    arg_string = []
    coordinates = []
    if len(sys.argv) == 4:
        arg_string = sys.argv[1:]
        for i in arg_string:
            coordinates.append(float(i))
    else:
        print('Please enter an x,y and z coordinate')
        exit()
    print(coordinates)
    try:
        flyTo(coordinates)
    except rospy.ROSInterruptException:
        pass
