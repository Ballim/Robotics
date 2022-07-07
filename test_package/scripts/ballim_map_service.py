#! /usr/bin/env python
# At this point it only gets a map of the occupancy map
import rospy
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import MapMetaData, OccupancyGrid, MapMetaData
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Empty, Int16MultiArray, Int16, Float32MultiArray
from rospy_tutorials.msg import Floats
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovarianceStamped
import random
import math
from math import radians, copysign, sqrt, pow, pi, atan2

raw_map = None
meta = None
initial_pose = (0,0)
res= None
origin= None

kp_distance = 1
ki_distance = 0.0
kd_distance = 0.0

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

class GotoPoint():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo(
                    "Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        
        global global_coordinates
        for index, coord in enumerate(global_coordinates):
            (position, rotation) = self.get_odom()

            last_rotation = 0
            linear_speed = 1  # kp_distance
            angular_speed = 1  # kp_angular

            (goal_x, goal_y) = coord

            if (index == len(global_coordinates)-1):
                goal_z = 0
            else:
                goal_z = atan2(global_coordinates[index+1][1] - goal_y, global_coordinates[index+1][0] - goal_x)
            
            print("Setting Goal Z", goal_z)
            #goal_z = np.deg2rad(goal_z)

            goal_distance = sqrt(pow(goal_x - position.x, 2) +
                                pow(goal_y - position.y, 2))
            # distance is the error for length, x,y
            distance = goal_distance
            previous_distance = 0
            total_distance = 0

            previous_angle = 0
            total_angle = 0

            print("Moving Toward Goal")
            while distance > 0.05:
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                #path_angle = error
                path_angle = atan2(goal_y - y_start, goal_x - x_start)

                if path_angle < -pi/4 or path_angle > pi/4:
                    if goal_y < 0 and y_start < goal_y:
                        path_angle = -2*pi + path_angle
                    elif goal_y >= 0 and y_start > goal_y:
                        path_angle = 2*pi + path_angle
                if last_rotation > pi-0.1 and rotation <= 0:
                    rotation = 2*pi + rotation
                elif last_rotation < -pi+0.1 and rotation > 0:
                    rotation = -2*pi + rotation

                diff_angle = path_angle - previous_angle
                diff_distance = distance - previous_distance

                distance = sqrt(pow((goal_x - x_start), 2) +
                                pow((goal_y - y_start), 2))

                control_signal_distance = kp_distance*distance + \
                    ki_distance*total_distance + kd_distance*diff_distance

                control_signal_angle = kp_angle*path_angle + \
                    ki_angle*total_angle + kd_distance*diff_angle

                move_cmd.angular.z = (control_signal_angle) - rotation
                #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                move_cmd.linear.x = min(control_signal_distance, 1)

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                last_rotation = rotation
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                previous_distance = distance
                total_distance = total_distance + distance
                #print("Current positin and rotation are: ", (position, rotation))

            (position, rotation) = self.get_odom()
            print("Current positin and rotation are: ", (position, rotation))

            print("reached :)   ^_^")

            print("Fixing the Angle from", rotation, goal_z)
            while abs(rotation - goal_z) > 0.05:
                (position, rotation) = self.get_odom()
                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = 0.5
                    else:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = -0.5
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = -0.5
                    else:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = 0.5
                self.cmd_vel.publish(move_cmd)
                #r.sleep()
        return

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


def map_callback(data):
    global raw_map
    raw_map = data.data

def map_data(data):
    global meta
    meta = data

def shape_array(data, meta):
    unshaped = np.array(data)
    shaped = np.reshape(unshaped,(-1,int(meta.width)))
    return shaped

def map_talker(map_arr):
    unique, counts = np.unique(map_arr,return_counts=True)
    results=np.column_stack((unique,counts))
    #print(results)

def initalPose(data):
    global initial_pose
    msg = data.pose.pose.position
    initial_pose = (msg.x, msg.y)
def map():
    rospy.Subscriber("move_base/global_costmap/costmap",OccupancyGrid,map_callback)
    rospy.Subscriber("map_metadata",MapMetaData,map_data)
    
    rospy.sleep(2)
    shaped_map = shape_array(raw_map, meta)

    global res
    res=meta.resolution
    #print("res")
    #print(res)
    global origin
    origin=meta.origin
    #print("origin")
    #print(origin)

    #print(shaped_map.shape)
    #print(shaped_map)
        # rospy.spin()
        # exit
    return shaped_map

def Dist(x1,y1,x2,y2):
    return math.sqrt(((x1-x2)**2)+((y1-y2)**2))

def GenerateRandomPoint(board):
    x=random.randint(1,board.shape[0]-1)
    y=random.randint(1,board.shape[1]-1)

    if (board[x,y]!=0):
        GenerateRandomPoint(board)
    
    return x,y

def GetClosestNode(Tree,RandomP):
    ShortestDist=9999999
    ShortestIndex=-1
    for i in range(len(Tree)):
        Distance=Dist(Tree[i][0],Tree[i][1],RandomP[0],RandomP[1])
        if Distance<ShortestDist:
            ShortestDist=Distance
            ShortestIndex=i
    return ShortestIndex

def CheckSurroundingClosest(Point,board,RandomP, destx,desty):
    check=[-5,0,5]
    ShortestCoords=(-1,-1)
    ShortestDist=9999999
    for k in check:
        for j in check:
            if (Point[0]>0 and Point[1]<board.shape[1]):
                if (Dist(Point[0]+k,Point[1]+j,destx,desty)<=20):
                    return (Point[0]+k,Point[1]+j), True
                if board[Point[0]+k,Point[1]+j]==0:
                    Distance=Dist(Point[0]+k,Point[1]+j,RandomP[0],RandomP[1])
                    if Distance<ShortestDist:
                        ShortestDist=Distance
                        ShortestCoords=(Point[0]+k,Point[1]+j)
    return ShortestCoords, False


def findshortestpath(grid,startx,starty,destx,desty):
    #print("finding optimal path")
    NewGrid=grid
    StartTree=[]
    Parents=[]

    StartTree.append([startx,starty])

    Parents.append(0)

    PathFound=False
    count=0
    # NewGrid[destx][desty]=2
    while(PathFound==False):
        randomp=GenerateRandomPoint(NewGrid)
        shortest=GetClosestNode(StartTree,randomp)
        closestNode=StartTree[shortest]

        NextNode, PathFound=CheckSurroundingClosest(closestNode,NewGrid,randomp,destx,desty)
        StartTree.append(NextNode)
        Parents.append(closestNode)
        NewGrid[NextNode]=1

        count+=1

    #print("Destination Reached")
    path=[]
    CurrentNode=NextNode
    Parent=closestNode
    while(CurrentNode!=[startx,starty]):
        NewGrid[CurrentNode]=5
        CurrentNode=Parent
        path.append(CurrentNode)
        for i in range(len(StartTree)):
            if StartTree[i]==CurrentNode:
                Parent=Parents[i]

    NewGrid[startx,starty]=10
    NewGrid[destx,desty]=10

    return path

def toRealCoords(x,y):
    real_x=x*res+origin.position.x
    real_y=y*res+origin.position.y
    return real_x,-real_y

def toPixCoords(x,y):
    y=-y
    pix_x=(x-origin.position.x)/res
    pix_y=(y-origin.position.y)/res
    return pix_x,pix_y

def GetFreeNodes(nodes,OMap):
    FreeNodes=nodes
    for i in range(len(nodes)):
        for k in range(len(nodes[i])):
            x,y=toPixCoords(nodes[i][k][0],nodes[i][k][1])
            if(OMap[x][y]!=0):
                FreeNodes[i][k]=([-1,-1])
    return FreeNodes


class QItem:
    def __init__(self, row, col, dist):
        self.row = row
        self.col = col
        self.dist = dist

def isValid(x, y, grid, visited,source):
    if ((x >= 0 and y >= 0) and
        (x < len(grid) and y < len(grid[0])) and
            (grid[x][y]<30 and grid[x][y]!=-1)  and (visited[x][y] == False)):
        return True
    return False

def opencheck(x, y, grid, visited,source):
    if(x==source.row):
        if(y>source.col):
            for i in range(y,source.col,-1):
                if(grid[x][i]>60 or grid[x][i]==-1):
                    #print("objdetect")
                    # visited[x][y] == False
                    return False
        else:
            for i in range(y,source.col):
                if(grid[x][i]>60 or grid[x][i]==-1):
                    #print("objdetect")
                    # visited[x][y] == False
                    return False
    if(y==source.col):
        if(x>source.row):
            for i in range(x,source.row,-1):
                if(grid[i][y]>60 or grid[i][y]==-1):
                    #print("objdetect")
                    # visited[x][y] == False
                    return False
        else:
            for i in range(x,source.row):
                if(grid[i][y]>60 or grid[i][y]==-1):
                    #print("objdetect")
                    # visited[x][y] == False
                    return False
    return True

def minDistance(grid,startx,starty,destx,desty):
    #print("starting BFS")
    source = QItem(startx,starty, 0)
    visited = [[False for _ in range(len(grid[0]))]
               for _ in range(len(grid))]
    
    dists=np.zeros((grid.shape[0],grid.shape[1]))
    
    queue = []
    queue.append(source)
    visited[source.row][source.col] = True
    dists[source.row][source.col]=source.dist
    parent_list=[]
    rate=8
    while len(queue) != 0:
        # #print("being While")
        source = queue.pop(0)
        # #print(len(queue))
        # Destination found;
        if (Dist(source.row,source.col,destx,desty) <= 20):
            #print("dest found")
            parent_list.append((destx,desty))
            current=(source.row,source.col)
            while(current[0]!= startx or current[1]!=starty):
                parent_list.append(current)

                lowest=9999
                temp=(0,0)
                scan1=[-rate,rate,0,0]
                scan2=[0,0,-rate,rate]
                for i in range(len(scan1)):
                    if(visited[current[0]+scan1[i]][current[1]+scan2[i]]==True):
                        # if(opencheck(current[0]+scan1[i],current[1]+scan2[i],grid,True, QItem(current[0],current[1],0))):
                        if(dists[current[0]+scan1[i]][current[1]+scan2[i]]<lowest):
                            lowest=dists[current[0]+scan1[i]][current[1]+scan2[i]]
                            temp=(current[0]+scan1[i],current[1]+scan2[i])

                current=temp
                # current=parents[current[0]][current[1]]
            return source.dist, parent_list


        # moving right
        if isValid(source.row, source.col + rate, grid, visited,source):
            queue.append(QItem(source.row, source.col + rate, source.dist + rate))
            visited[source.row][source.col + rate] = True
            dists[source.row][source.col+rate] = source.dist + rate

        # moving left
        if isValid(source.row, source.col - rate, grid, visited,source):
            queue.append(QItem(source.row, source.col - rate, source.dist + rate))
            visited[source.row][source.col - rate] = True
            dists[source.row][source.col-rate] = source.dist + rate

        # moving down
        if isValid(source.row + rate, source.col, grid, visited,source):
            queue.append(QItem(source.row + rate, source.col, source.dist + rate))
            visited[source.row + rate][source.col] = True
            dists[source.row+rate][source.col] = source.dist + rate

        # moving up
        if isValid(source.row - rate, source.col, grid, visited,source):
            queue.append(QItem(source.row - rate, source.col, source.dist + rate))
            visited[source.row - rate][source.col] = True
            dists[source.row-rate][source.col] = source.dist + rate
 
    return -1

def Gradient(x1,x2,y1,y2):
    if(x1==x2):
        return 99999
    return (y2-y1)/(x2-x1)

def getGradients(Coords):
    Gradients=[]
    for i in range(len(Coords)-2):
        Gradients.append(Gradient(Coords[i][0],Coords[i+1][0],Coords[i][1],Coords[i+1][1]))
    return Gradients

def SmoothPath(Coords,Grads):
    SmoothPath=[]
    SmoothPath.append(Coords[0])
    CurrentGrad=Grads[0]
    TempCoord=None
    for i in range(1,len(Grads)):
        if (Grads[i]==CurrentGrad):
            TempCoord=Coords[i+1]
        else:
            SmoothPath.append(TempCoord)
            TempCoord=Coords[i+1]
            CurrentGrad=Grads[i]
    if(TempCoord!=None):
        SmoothPath.append(TempCoord)

    return SmoothPath

def planpath(OccMap,DestCoords):
    check1=[-1,0,1]
    check2=[-1,0,1]
    new_map=OccMap
    OccMap=np.transpose(np.flip(OccMap,0))
 
    #print(new_map.shape)
 
    unique, counts = np.unique(new_map,return_counts=True)
    results=np.column_stack((unique,counts))
    #print(results)
    #print(OccMap)
    np.savetxt('Map.csv',OccMap,delimiter=',')

    #print("converting pix coords to world")
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, initalPose)
    print('initial_pose', initial_pose)
    rospy.sleep(1)
    
    x,y=toPixCoords(initial_pose[0],initial_pose[1])
    x=round(x)
    y=round(y)
    #print(x)
    #print(y)
    #print(OccMap[int(x)][int(y)])

    
    

    goalx,goaly=toPixCoords(DestCoords[0],DestCoords[1])
    #print("dest coords")
    #print(DestCoords[0],DestCoords[1])
    #print("dest map coords")
    goalx=round(goalx)
    goaly=round(goaly)
    #print(int(goalx), int(goaly))
    #print("Map Value")
    #print(OccMap[int(goalx)][int(goaly)])
    #print("BFS")
    mind,coords=minDistance(OccMap,int(x),int(y),int(goalx),int(goaly))
    #print(mind)
    xs=[]
    ys=[]
    for i in coords:
        xs.append(i[0])
        ys.append(i[1])
    np.savetxt("x.csv",xs,delimiter=',')
    np.savetxt("y.csv",ys,delimiter=',')
    #print([i[0] for i in coords])
    #print(",")
    #print([i[1] for i in coords])
    #print("BFS Convert")
    
    OrderedCoords=[]
    for i in range(len(coords)):
        cx,cy=toRealCoords(coords[i][0],coords[i][1])
        # #print(round(cx,4),round(cy,4))
        OrderedCoords.append((cx,cy))
    OrderedCoords.reverse()
    #print(OrderedCoords)
    #print("Coords len:"+str(len(OrderedCoords)))
    
    #print("Getting gradients:")
    Grads=getGradients(OrderedCoords)
    #print("Grads len:"+str(len(Grads)))
    #print(Grads)
    
    #print("Smoothing Path:")
    print("Ordered Coords: ", OrderedCoords)
    print("Gradients: ", Grads)
    SmoothP=SmoothPath(OrderedCoords,Grads)
    SmoothP.append((DestCoords[0],DestCoords[1]))
    #SmoothP.pop(0)
    return SmoothP
    

if __name__ == '__main__':
    try:
        rospy.init_node('make_map', anonymous=False)
        s_map=map()
        map_talker(s_map)
        print("Please enter the coordinates of your destination:")
        destx= input("x: ")
        desty= input("y: ")
        dest=[float(destx),float(desty)]
        global_coordinates = planpath(s_map,dest)
        rospy.loginfo(global_coordinates)
        GotoPoint()
        print("Ya made it ma boy")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass