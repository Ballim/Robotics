#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties
# from tf.transformations import euler_from_quarternion
from time import sleep
import math

TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
TurtlePos=TBotPos('mobile_base',"")
print(TurtlePos)




def MoveStraight():
    print("moving")
    rospy.init_node('topic_publisher')
    foward = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    # rate=rospy.Rate(10)
    # rate.sleep()
    # while(foward.get_num_connections()<1):
    #     sleep(0.5)
    rate=rospy.Rate(2)
    move=Twist()
    move.linear.x=1.0
    # move.linear.y=0.0
    # move.linear.z=0.0
    # move.angular.x=0.0
    # move.angular.y=0.0
    move.angular.z=0.5
    for i in range(5):
        foward.publish(move)
        rate.sleep()

# MoveStraight()



def GoTo(DestCoord):
    # rospy.init_node('pid', anonymous=True)
    # rate=rospy.Rate(10)

    # takeoff=rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    # fly=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    # land=rospy.Publisher('ardrone/land',Empty,queue_size=10)

    rospy.init_node('topic_publisher')
    foward = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rate=rospy.Rate(10)

    # while ((takeoff.get_num_connections() < 1) or (land.get_num_connections() < 1)):
    #     sleep(0.5)
    
    # takeoff.publish()
    # sleep(2.5)
    # rate.sleep()

    #pid
    ki=0
    kd=0
    kp=0.5
    kiz=0
    kdz=0
    kpz=0.1
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
    

    # TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    # pos=TBotPos('mobile_base',"")
    # print(pos.pose.orientation.z)
    # drone_coords=np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z])
    # drone_ang=pos.pose.orientation.z
    # angle=math.atan2(dest_coords[1]-drone_coords[1],dest_coords[0]-drone_coords[0])*180/math.pi
    # drone_ang_dest=angle
    # drone_angle=pos.pose.orientation.z*180
    # ang_error=drone_ang_dest-drone_angle

    # while(abs(ang_error)>0.1):
    #     TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    #     pos=TBotPos('mobile_base',"")
    #     print(pos.pose.orientation.z)
    #     drone_coords=np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z])
    #     drone_ang=pos.pose.orientation.z
    #     angle=math.atan2(dest_coords[1]-drone_coords[1],dest_coords[0]-drone_coords[0])*180/math.pi
    #     drone_ang_dest=angle
    #     drone_angle=pos.pose.orientation.z*180
    #     ang_error=drone_ang_dest-drone_angle

    #     move=Twist()
    #     move.linear.x=0
    #     move.linear.y=0
    #     move.linear.z=0
    #     move.angular.x=0
    #     move.angular.y=0
    #     move.angular.z=ang_error/10
    #     foward.publish(move)
    #     # rate.sleep()

    while((np.linalg.norm(dest_coords-drone_coords,2)>0.075) or (iter==0)):
        TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        pos=TBotPos('mobile_base',"")
        print(pos.pose.orientation.z)
        drone_coords=np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z])
        drone_ang=pos.pose.orientation.z
        angle=math.atan2(dest_coords[1]-drone_coords[1],dest_coords[0]-drone_coords[0])*180/math.pi
        
        # orientations=pos.pose.orientation
        # tutrle_orientation=orientations.z
        # (roll)
        


        drone_ang_dest=angle
        drone_angle=pos.pose.orientation.z*180
        # if(drone_ang_dest>drone_angle):
        #     angle=drone_ang_dest-drone_angle
        # else:
        #     angle=drone_angle-drone_ang_dest
        print(drone_coords)
        print(dest_coords)
        
        
        ang_error=drone_ang_dest-drone_angle
        if (ang_error>180):
            ang_error=360-ang_error
        elif(ang_error<-180):
            ang_error=360+ang_error

        error=dest_coords-drone_coords
        print(error)

        error_sum_ang+=ang_error
        error_add+=error

        error_dif_ang = abs(ang_error - error_prev_ang)
        error_diff=abs(error-error_prev)

        error_prev=error
        error_dif_ang=ang_error
        dist=math.sqrt((dest_coords[0]-drone_coords[0])**2+(dest_coords[1]-drone_coords[1])**2)
        U=kp*error + ki*error_add + kd*error_diff

        # if(angle<0):
        #     Uang=-0.1
        #     U[0]=0
        #     if(angle>-3):
        #         Uang=0
        #         U=kp*error + ki*error_add + kd*error_diff
        # else:
        #     Uang= 0.1
        #     U[0]=0
        #     if(angle<3):
        #         Uang=0
        #         U=kp*error + ki*error_add + kd*error_diff

        Uang= kpz*ang_error 
        
        print(dist)
        print(angle)
        print(drone_angle)
        print(ang_error)
        # if(abs(ang_error)>1):
        #     linx=0
        #     # Uang=Uang/abs(Uang)*kpz
        # else:
        # if(abs(ang_error)<1):
        #     linx=abs(dist*kp)
        #     # Uang=0
        # else:
        #     # linx=0
        #     while(ang_error>0.001):
        #         TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        #         pos=TBotPos('mobile_base',"")
        #         print(pos.pose.orientation.z)
        #         drone_coords=np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z])
        #         drone_ang=pos.pose.orientation.z
        #         angle=math.atan2(dest_coords[1]-drone_coords[1],dest_coords[0]-drone_coords[0])*180/math.pi
        #         drone_ang_dest=angle
        #         drone_angle=pos.pose.orientation.z*180
        #         ang_error=drone_ang_dest-drone_angle

        #         move=Twist()
        #         move.linear.x=0
        #         move.linear.y=0
        #         move.linear.z=0
        #         move.angular.x=0
        #         move.angular.y=0
        #         move.angular.z=ang_error/10
        #         foward.publish(move)
        #             # Uang=0
        # # linx=0
        linx=abs(dist*kp)
        if(abs(ang_error)>45):
            linx=0
            # Uang=Uang/2
            # Uang=2
        liny=0
        linz=0
        move=Twist()
        move.linear.x=linx
        move.linear.y=liny
        move.linear.z=linz
        move.angular.x=0
        move.angular.y=0
        move.angular.z=Uang
        foward.publish(move)
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