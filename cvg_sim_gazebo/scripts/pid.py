#! /usr/bin/env python
# import queue
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties
from time import sleep
import numpy as np

# rospy.init_node('pid', anonymous=True)


# # rospy.wait_for_service('gazebo/get_model_state')
# def CurrentPos():
#     try:
#         # rosservice call gazebo/get_model_properties '{model_name: rrbot}'

#         model_coordinates=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
#         # while model_coordinates.get_num_connections() < 1:
#         #     rospy.loginfo_throttle(2, "Waiting for subscribers on /ardrone/takeoff ..")
#         #     rospy.sleep(0.1)
#         pos=model_coordinates("quadrotor","")
#         print (pos.success)
#         print((pos.pose.position.x))
#         print((pos.pose.position.y))
#         print((pos.pose.position.z))
#         return pos.pose.position.x,pos.pose.position.y,pos.pose.position.z
#     except:
#         print('failed')

# destx= input("x: ")
# desty= input("y: ")
# destz= input("z: ")
# print(destx)
# print(desty)
# print(destz)
# x,y,z=CurrentPos()
# error=np.array([x-destx,y-desty,z-destz])
# integralerror=np.sum(error)

# print(error)
# print(integralerror)

# twist=Twist()
# twist.linear.x

def GoTo(DestCoord):
    rospy.init_node('pid', anonymous=True)
    rate=rospy.Rate(10)

    takeoff=rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    fly=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    land=rospy.Publisher('ardrone/land',Empty,queue_size=10)
    while ((takeoff.get_num_connections() < 1) or (land.get_num_connections() < 1)):
        sleep(0.5)
    
    takeoff.publish()
    sleep(2.5)
    rate.sleep()

    #pid
    ki=0
    kd=0
    kp=0.5
    kiz=0.3
    kdz=0
    kpz=0.5
    error_add=[0,0,0]
    error_prev=[0,0,0]
    error_diff=[0,0,0]
    error_sum_ang=0
    error_prev_ang=0
    error_dif_ang=0
    drone_ang_dest=0
    drone_coords=[0,0,0]
    dest_coords=np.array(DestCoord)
    iter=0

    while((np.linalg.norm(dest_coords-drone_coords,2)>0.075) or (iter==0)):
        model_coords=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        pos=model_coords('quadrotor',"")
        print(pos)
        drone_coords=np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z])
        drone_ang=pos.twist.angular.z

        print(drone_coords)
        print(dest_coords)

        ang_error=drone_ang_dest-drone_ang
        error=dest_coords-drone_coords
        print(error)

        error_sum_ang+=ang_error
        error_add+=error

        error_dif_ang = abs(ang_error - error_prev_ang)
        error_diff=abs(error-error_prev)

        error_prev=error
        error_dif_ang=ang_error

        U=kp*error + ki*error_add + kd*error_diff
        Uang= kpz*ang_error + kiz*error_sum_ang + kdz*error_dif_ang

        print(U)
        linx=U[0]
        liny=U[1]
        linz=U[2]
        move=Twist()
        move.linear.x=linx
        move.linear.y=liny
        move.linear.z=linz
        move.angular.x=0
        move.angular.y=0
        move.angular.z=Uang
        fly.publish(move)
        rate.sleep()
        iter+=1

    rate.sleep()





if __name__ == '__main__':
    destx= input("x: ")
    desty= input("y: ")
    destz= input("z: ")
    dest=[float(destx),float(desty),float(destz)]
    try:
        GoTo(dest)
    except rospy.ROSInterruptException:
        pass