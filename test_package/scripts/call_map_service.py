#! /usr/bin/env python
# At this point it only gets a map of the occupancy map
from audioop import ratecv
from tabnanny import check
import rospy
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import MapMetaData, OccupancyGrid, MapMetaData
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Empty, Int16MultiArray, Int16
from rospy_tutorials.msg import Floats
import random
import math

raw_map = None
meta = None

res= None
origin= None


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
    print(results)


def map():
    rospy.init_node('make_map', anonymous=True)
    rospy.Subscriber("move_base/global_costmap/costmap",OccupancyGrid,map_callback)
    rospy.Subscriber("map_metadata",MapMetaData,map_data)
    rospy.sleep(2)
    # while not rospy.is_shutdown():
    shaped_map = shape_array(raw_map, meta)
    global res
    res=meta.resolution
    print("res")
    print(res)
    global origin
    origin=meta.origin
    print("origin")
    print(origin)
    print(shaped_map.shape)
    print(shaped_map)

    return shaped_map

def Dist(x1,y1,x2,y2):
    return math.sqrt(((x1-x2)**2)+((y1-y2)**2))


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

# checking where move is valid or not
def isValid(x, y, grid, visited,source):
    if ((x >= 0 and y >= 0) and
        (x < len(grid) and y < len(grid[0])) and
            (grid[x][y]<30 and grid[x][y]!=-1)  and (visited[x][y] == False)):
        return True
    return False



def minDistance(grid,startx,starty,destx,desty):
    print("starting BFS")
    source = QItem(startx,starty, 0)
 
    # To maintain location visit status
    visited = [[False for _ in range(len(grid[0]))]
               for _ in range(len(grid))]
    
    dists=np.zeros((grid.shape[0],grid.shape[1]))
    # applying BFS on matrix cells starting from source
    queue = []
    queue.append(source)
    visited[source.row][source.col] = True
    dists[source.row][source.col]=source.dist
    parent_list=[]
    rate=8
    while len(queue) != 0:
        # print("being While")
        source = queue.pop(0)
        # print(len(queue))
        # Destination found;
        if (Dist(source.row,source.col,destx,desty) <= 20):
            print("dest found")
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
 
        

        if isValid(source.row + rate, source.col, grid, visited,source):
            queue.append(QItem(source.row + rate, source.col, source.dist + rate))
            visited[source.row + rate][source.col] = True
            dists[source.row+rate][source.col] = source.dist + rate

        # moving up
        if isValid(source.row - rate, source.col, grid, visited,source):
            queue.append(QItem(source.row - rate, source.col, source.dist + rate))
            visited[source.row - rate][source.col] = True
            dists[source.row-rate][source.col] = source.dist + rate
 
        # moving down
        
 
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
    for i in range(1,len(Grads)):
        if (Grads[i]==CurrentGrad):
            TempCoord=Coords[i+1]
        else:
            SmoothPath.append(TempCoord)
            TempCoord=Coords[i+1]
            CurrentGrad=Grads[i]
    SmoothPath.append(TempCoord)

    return SmoothPath

def planpath(OccMap,DestCoords):
    print("Planning Optimal Path...")
    check1=[-1,0,1]
    check2=[-1,0,1]
    new_map=OccMap
    OccMap=np.transpose(np.flip(OccMap,0))
    print("Shape of Map Array:"+str(new_map.shape))
    unique, counts = np.unique(new_map,return_counts=True)
    results=np.column_stack((unique,counts))
    print("Value counts in map:")
    print(results)
    # print(OccMap)
    np.savetxt('Map.csv',OccMap,delimiter=',')

    print("converting pix coords to world")
    x,y=toPixCoords(0,0)
    x=round(x)
    y=round(y)
    print("Starting map coordinates :")
    print("Pixel coords: "+str(x)+","+str(y))
    print("Map Value at starting point: "+str(OccMap[int(x)][int(y)]))


    
    

    goalx,goaly=toPixCoords(DestCoords[0],DestCoords[1])
    print("dest coords")
    print(DestCoords[0],DestCoords[1])
    print("dest map coords")
    goalx=round(goalx)
    goaly=round(goaly)
    print(int(goalx), int(goaly))
    print("Map Value")
    print(OccMap[int(goalx)][int(goaly)])
    print("BFS")
    mind,coords=minDistance(OccMap,int(x),int(y),int(goalx),int(goaly))
    print(mind)
    xs=[]
    ys=[]
    for i in coords:
        xs.append(i[0])
        ys.append(i[1])
    np.savetxt("x.csv",xs,delimiter=',')
    np.savetxt("y.csv",ys,delimiter=',')
    print([i[0] for i in coords])
    print(",")
    print([i[1] for i in coords])
    print("BFS Convert")
    
    OrderedCoords=[]
    for i in range(len(coords)):
        cx,cy=toRealCoords(coords[i][0],coords[i][1])
        OrderedCoords.append((round(cx,4),round(cy,4)))
    OrderedCoords.reverse()
    print(OrderedCoords)
    print("Coords len:"+str(len(OrderedCoords)))
    
    print("Getting gradients:")
    Grads=getGradients(OrderedCoords)
    print("Grads len:"+str(len(Grads)))
    print(Grads)
    
    print("Smoothing Path:")
    SmoothP=SmoothPath(OrderedCoords,Grads)
    SmoothP.append((DestCoords[0],DestCoords[1]))
    print(SmoothP)

    Smoothx=[]
    Smoothy=[]
    for i in SmoothP:
        Smoothx.append(i[0])
        Smoothy.append(i[1])
    np.savetxt("Smoothx.csv",xs,delimiter=',')
    np.savetxt("Smoothy.csv",ys,delimiter=',')
    print("Smooth Path len:"+str(len(SmoothP)))

    
    return  

if __name__ == '__main__':
    try:
        s_map=map()
        map_talker(s_map)
        print("Please enter the coordinates of your destination:")
        destx= input("x: ")
        desty= input("y: ")
        destz= input("z: ")
        dest=[float(destx),float(desty),float(destz)]
        planpath(s_map,dest)
    except rospy.ROSInterruptException:
        pass