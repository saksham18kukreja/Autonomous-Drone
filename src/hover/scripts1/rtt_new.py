#!/usr/bin/env python
#define the environment the obstacles and boundary, sample space
#define the start node and goal node
#define the start node as the parent node and generate a random point in the sample space which should be in the limit set as delta
#check if the point is not colliding with any obstacle 
#join the new node with the node nearest to the node and check that the line should not be crossing any obstacle
#save the tree and repeat the itertions until the goal node is reached
#return the path connecting the parent of each node to the start node

####### New algorithm based on filtering nodes based on volmetric search instead of calculating distance with every point
####### Check if this algorithm produce lesser time than the previous one.

import numpy as np
import math
import random
import time
from sklearn.neighbors import KDTree
from Extra import extra

#defining the node
class Node(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

#defining the environment 
class RRT(object):
    def __init__(self,obstacle,start,goal,xrange,yrange,zrange):
        self.start = start
        self.start.parent = None
        self.goal = goal
        self.Xrange = xrange #boundary
        self.Yrange = yrange
        self.Zrange = zrange
        self.delta = 1 #the distance between two nodes
        self.checkNum = 0.05
        self.points = []
        self.path = []
        self.optimal = []
        self.obstacle_tree = obstacle
        #self.body = self.aicraft_points()
        self.drone_len = 0.3
        self.smooth = extra()
        self.smooth_path = []

    #distance function
    def getDistance(self,p1,p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)


    #nearest node to the new node
    def getNearest(self,point):
        min_distance = self.getDistance(point,self.points[0])
        nearest = self.points[0]
        for q in self.points:
            if self.getDistance(point,q) < min_distance:
                min_distance = self.getDistance(point,q)
                nearest = q
        return nearest      

    #generating new node near the nearest node within the distance of delta
    def getNew(self,rand,near):
        scale = float(self.delta/self.getDistance(rand,near))
        new_x = near.x + scale*(rand.x - near.x)
        new_y = near.y + scale*(rand.y - near.y)
        new_z = near.z + scale*(rand.z - near.z)
        new = self.checkBoundary(Node(new_x,new_y,new_z))
        return new

    #check if the new node is within the boundary 
    def checkBoundary(self,point):
        if point.x > self.Xrange:
            point.x = self.Xrange
        if point.y > self.Yrange:
            point.y = self.Yrange
        if point.z > self.Zrange:
            point.z = self.Zrange
        return point

    #return path
    def return_path(self):
        self.path.append(self.goal)
        #path_new =[]
        while (self.path[-1].parent != None):
            self.path.append(self.path[-1].parent)
            
        self.smooth_path = self.generate_bezier(self.path[::-1])
        return self.smooth_path, self.path[::-1]
        #return self.path[::-1]
        #return self.optimal

    #check the equivalence of two points:
    def check_equal(self,p1,p2):
        if (self.getDistance(p1,p2)<=self.drone_len):
            return False
        return True    
    
    # check if the new node is inside a obstacle or not
    def check_obstacle(self,point):
        p = np.array([(point.x, point.y, point.z)])
        nearest_pt = self.obstacle_tree.query_radius(p,r=0.8,count_only=True)
        if nearest_pt[:1] != 0:
            return False
        else:
            return True    
            
        

    # check if the line joining the new node and nearest node does not cross an obstacle
    def check_line(self,p1,p2):
        count = self.getDistance(p1,p2)/self.checkNum
        dx = (p2.x-p1.x)/count
        dy = (p2.y-p1.y)/count
        dz = (p2.z-p1.z)/count
        for i in range(int(count)):
            temp = Node(i*dx+p1.x,i*dy+p1.y,i*dz+p1.z)
            if self.check_obstacle(temp) == False:
                return False  
        return True


    #function to generate points representing the body of the aircraft as a cuboid
    def aicraft_points(self):
        aircraft_body = []
        k=0.2
        dim = np.linspace(-k,k,10)
        
        for i in dim:
            for j in dim:
                for k in dim:
                    aircraft_body.append((i,j,k))
        return aircraft_body


    #function to check if the points lie in the obstacle
    def check_aircraft_obstacle(self,center):
        body_points = [(center.x + point[0],center.y + point[1],center.z + point[2])for point in self.body]
        for point in body_points:
            if self.check_obstacle(point)==False:
                return False
            else:
                continue
        return True        


    # function to smooth the path      
    def getOptimal(self,path):
        #self.return_path()
        optimal = path[::-1]
        for i in range(100):
            minLength = len(optimal)
            pick1 = random.randint(1,minLength-1)
            pick2 = random.randint(1,minLength-1)
            if (pick1 != pick2) and (self.check_line(optimal[pick1],optimal[pick2])):
                if pick1 > pick2:
                    del optimal[pick2+1:pick1]
                elif pick1 < pick2:
                    del optimal[pick1+1:pick2]
        return optimal

    def generate_bezier(self,path):
        bezier_curve = []
        # for i in range(len(path)):
        #     points.append([path[i].x,path[i].y,path[i].z])
        points = [[path[i].x,path[i].y,path[i].z]for i in range(len(path))]
        path_smooth = self.smooth.evaluate_bezier(np.array(points))
        
        for i in range(len(path_smooth)):
            bezier_curve.append(Node(path_smooth[i][0], path_smooth[i][1], path_smooth[i][2],))
        
        return bezier_curve



    # Main function calling the other functions    
    def main(self):
        print('inside path_planner')
        self.points.append(self.start)
        iter=0
        number_of_iterations = 150
        #print(self.Obstacles)

        while iter<=number_of_iterations:
            rand = Node(np.random.uniform(0,self.Xrange),np.random.uniform(0,self.Yrange),np.random.uniform(0,self.Zrange))
            near = self.getNearest(rand)
            new = self.getNew(rand,near)

         #if there is a obstacle between them
         # false means there is an obstacle
            if (self.check_obstacle(new) == False): 
                continue

        #if the line connecting the two points passes through the obstacle 
            if (self.check_line(near,new) == False):
                continue
               
        #if there is no obstacle between the node and the goal and the disance is less than delta
            if (self.getDistance(new,self.goal) <= self.delta) and self.check_line(new,self.goal):
                self.goal.parent = new
                self.points.append(self.goal)
                new.parent = near
                self.points.append(new)
                return self.return_path()
 
       

        # add the new point in the list of visited points
            new.parent = near
            self.points.append(new)
            iter = iter+1

        print('No path found')
        return None
               

if __name__ == "__main__":
    obstacle1 = Node(4,4,4)
    obstacle2 = Node(3,3,3)
    obstacle = [obstacle1,obstacle2]
    start = Node(0,0,0)
    goal = Node(5,5,5)
    xrange = 7
    yrange = 7
    zrange = 7
    t1 = time.time()
    obs = RRT(obstacle,start,goal,xrange,yrange,zrange)
    t2 = time.time()
    path = obs.main()
    print(path)
    print(t2-t1)
    #obs.path_plot(path)   
