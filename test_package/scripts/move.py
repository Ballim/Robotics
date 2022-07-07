import rospy
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties
from time import sleep

TBotPos=rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
TurtlePos=TBotPos('mobile_base',"")
print(TurtlePos)