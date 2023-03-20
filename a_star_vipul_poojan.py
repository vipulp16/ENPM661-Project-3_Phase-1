# ENPM661
# UID : 119395547
# Project 3- Phase 1

# Github Repo:  https://github.com/vipulp16/ENPM661-Project-3_Phase-1

### Importing necessary libraries
import numpy as np
import cv2
import math
import sys
import copy
import time

# Defining the size of canvas
w = 600                          # width of canvas
h = 250                          # height of canvas

# Define output canvas
canvas = np.zeros((250, 600, 3))

# Define Colors
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
white = (255, 255, 255)
yellow = (0, 255, 255)

# def display_on_canvas():
#     canvas = cv2.flip(canvas,0)
#     cv2.imshow('Output',canvas)
#     cv2.waitKey(100) 
#     canvas = cv2.flip(canvas,0)

# Defining a function to calculate distance cost between start to goal node
def C2G(node):
    d = math.sqrt((node[0] - x2)**2 + (node[1] - y2)**2)
    return d

# Determing the order of node
def check_order(opennode_list):
    min = math.inf
    minvalue_node = None
    for node in opennode_list:
        # Checking for least value of C2C+C2G 
        if opennode_list[node][0] + opennode_list[node][1] < min:
            min = copy.deepcopy(opennode_list[node][0])+copy.deepcopy(opennode_list[node][1])
            minvalue_node = node
    return minvalue_node

# Defining the function to determine motion of robot
def action(active_node,angle_change):
    x_l = copy.deepcopy(active_node[0])
    y_l = copy.deepcopy(active_node[1])
    next_ang = active_node[2] - angle_change
    while next_ang >= 360:
        next_ang = next_ang - 360
    while next_ang <= -360:
        next_ang = next_ang + 360
    # Calculating the new positions
    x_n = int(x_l + M*math.cos(math.radians(next_ang)))
    y_n = int(y_l + M*math.sin(math.radians(next_ang)))
    # creating list of path points
    btwn_pnts = inpath_pnts(active_node,(x_n,y_n))
    # Checking for points in obstacle spaces
    for node in btwn_pnts:
        if (x_n >= canvas.shape[1] or x_n<0 or y_n >= canvas.shape[0] or y_n<0):
            stat = False
            new_node = (x_l,y_l,active_node[2])
            return [stat, new_node]
        if all(canvas[node[1],node[0]] == yellow) or all(canvas[node[1],node[0]] == red):
            stat = False
            new_node = (x_l,y_l,active_node[2])
            return [stat,new_node]
    stat = True
    new_node = (x_n,y_n,next_ang)
    return [stat, new_node]

# Defining function to backtrack from goal node to start node
def backtracking(closednode_list):
    path = [goal_node]
    while True:
        parent = copy.deepcopy(closednode_list[path[0]][2])
        if parent == None:
            break
        else:
            path.insert(0,parent)
    return path
    
# Finding the points in between the nodes
def inpath_pnts(n1,n2):
    list = []
    if n2[0] != n1[0]:
        for i in range(abs(n2[0]-n1[0])+1):
            if n2[0]>n1[0]:
                list.append([int(n1[0]+i),int(((n2[1]-n1[1])/(n2[0]-n1[0]))*i+n1[1])])
            else:
                list.append([int(n2[0]+i),int(((n2[1]-n1[1])/(n2[0]-n1[0]))*i+n2[1])])
    if n2[1] != n1[1]:
        for j in range (abs(n2[1]-n1[1])+1):
            if n2[1]>n1[1]:
                list.append([int(((n2[0]-n1[0])/(n2[1]-n1[1]))*j + n1[0]), int(n1[1]+j)])
            else:
                list.append([int(((n2[0]-n1[0])/(n2[1]-n1[1]))*j + n2[0]), int(n2[1]+j)])
    points = []
    for n in list:
        if n not in points:
            points.append(n)
    return points

# Define Obstacle space
for x in range(canvas.shape[1]): 
    for y in range(canvas.shape[0]):
        # Triangle 
        if x>= 460 and (y-2*x+895)>=0 and (y+2*x-1145)<=0:
            canvas[y,x] = red
        # Hexagon
        if x>=(300-75*math.cos(np.deg2rad(30))) and x<=(300+75*math.cos(np.deg2rad(30))) and (y-0.58*x-26.79)<=0 and (y+0.58*x-373.21)<=0 and (y-0.58*x+123.21)>=0 and (y+0.58*x-223.21)>=0:
            canvas[y,x] = red
        # Top Rectangle
        if x>=100 and x<=150 and y>=150 and y<=h: 
            canvas[y,x] = red
        # Bottom Rectangle
        if (100)<=x and x<=(150) and 0<=y and y<=(100): 
            canvas[y,x] = red
            

# display map with original obstracles            
# display_on_canvas()
canvas = cv2.flip(canvas,0)
cv2.imshow('Output',canvas)
cv2.waitKey(100) 
canvas = cv2.flip(canvas,0)

# Taking clearance from user (clearance = 5)
while True:
    clearance = int(input('Enter the clearance between the robot and obstacles:  \n'))
    # Taking radius of robot from the user
    radius = int(input('Enter the radius of robot:   \n'))
    if clearance<0:
        print("Enter positive value for clearance!!!")
        sys.exit(0)
    elif radius<0:
        print("Enter positive value for radius!!!")
        sys.exit(0)
    else:
        break
        
# Adding clearance and radius of robot to canvas
new_canvas = np.zeros((250,600,3))
# Applying clearance and radius to the obstacle
for x in range(canvas.shape[1]):
    for y in range(canvas.shape[0]):
        if all(canvas[y,x] == red): 
            cv2.circle(new_canvas, (x,y), clearance+radius, yellow, -1 )

# Applying clearance to the boundary
for x in range(canvas.shape[1]):
    for y in range(canvas.shape[0]):
        if (x<=0 or x>=599 or y<=0 or y>=249): 
            cv2.circle(new_canvas, (x,y), clearance, yellow, -1 )            
            
for x in range(canvas.shape[1]):
    for y in range(canvas.shape[0]):
        if all(canvas[y,x] == red):
            new_canvas[y,x] = red
            
canvas = copy.deepcopy(new_canvas)
# Display canvas
# display_on_canvas()
canvas = cv2.flip(canvas,0)
cv2.imshow('Output',canvas)
cv2.waitKey(100) 
canvas = cv2.flip(canvas,0)

while True:
    # Taking the input for step size
    M = int(input('Enter the step size to be taken:(1-10)  \n'))
    # Taking input for the Start Node
    x1 = int(input('Enter x co-ordinate of start Node(0-600)\n'))
    y1 = int(input('Enter y co-ordinate of start Node(0-250)\n'))
    start_angle = int(input('Enter start angle (multiple of 30):    \n'))
    while start_angle >= 360:
        start_angle = start_angle - 360
    while start_angle <= -360:
        start_angle = start_angle + 360
    # Taking input for Goal Node
    x2 = int(input('Enter x co-ordinate of Goal Node(0-600)\n'))
    y2 = int(input('Enter y co-ordinate of Goal Node(0-250)\n'))
    goal_angle = int(input('Enter goal angle:    \n'))
    while goal_angle >= 360:
        goal_angle = goal_angle - 360
    while goal_angle <= -360:
        goal_angle = goal_angle + 360
    
    # Checking if step is in range
    if M<1 or M> 10:
        print("Enter step value in the range of 1-10!!!")
        sys.exit(0)
        
    # Checking for start node in canvas space
    elif (x1<0) or x1 >= 600 or y1 < 0 or y1 >= 250:
        print("Start Node is out of canvas, Enter Valid input!!!!")
        sys.exit(0)
    elif all(canvas[y1,x1]==yellow) or all(canvas[y1,x1]==red):
        print('Start node is in obstracle, please try again')
        sys.exit(0)
        
    # Checking if start angle is a multiple of 30
    elif start_angle %30 != 0:
        print('start angle is invalid, please enter a multiple of 30.')
        sys.exit(0)
        
    # Checking for goal node in canvas space
    elif (x1<0) or x1>600 or y1<0 or y1>250:
        print("Goal Node is out of canvas, Enter Valid input!!!!")
        sys.exit(0)
    elif all(canvas[y2,x2]==yellow) or all(canvas[y2,x2]==red):
        print('Goal node is in obstracle, please try again')
        sys.exit(0)
        
    # Checking if goal angle is a multiple of 30
    elif goal_angle %30 != 0:
        print('Goal angle is invalid, please enter a multiple of 30.')
        sys.exit(0)
    else:
        break
    
begin = time.time()

# storing the start and goal nodes
start_node =(x1, y1, start_angle)
goal_node = (x2, y2, goal_angle)

# Defining the list to store currently open nodes
opennode_list = {}

# Defining list to store explored nodes
closednode_list = {}

# weigt for A* algo
wt = 1

# Add start node to the list to start exploration
opennode_list[start_node] = [0, C2G(start_node), None]

# Creating a loop to run A* Algorithm and reach goal node
while opennode_list != {}:
    #Define current live node 
    active_node = check_order(opennode_list)
    c2c = copy.deepcopy(opennode_list[active_node][0])
    # Adding the node to closed list and removing from open list
    closednode_list[active_node] = opennode_list.pop(active_node)
    # Check if the active node is goal node 
    # Also applying threshold of 1.5
    if (math.sqrt((active_node[0] - goal_node[0])**2 + (active_node[1] - goal_node[1])**2) <=1.5) and (abs(goal_node[2]-active_node[2]) <= 30):
        # If the goal is reached:
        print("Goal Reached!!!")
        print("Finding the Shortest path to goal node!!.......")
        goal_node = active_node
        # Start backtracking:
        path = backtracking(closednode_list)
        print(" Path Found!! \nDisplaying the path on Canvas!!....")
        # Display the path on canvas
        for i in range(len(path)-1):
            cv2.line(canvas,(path[i][0],path[i][1]),(path[i+1][0],path[i+1][1]), (0,0,255), 1)
            canvas = cv2.flip(canvas,0)
            cv2.imshow('Output',canvas)
            cv2.waitKey(10) 
            canvas = cv2.flip(canvas,0)
            #display_on_canvas()
        # Ouptu screen
        canvas = cv2.flip(canvas,0)
        cv2.imshow("Output",canvas)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        break                                                                                              
    # if the goal node is not reached applying action:
    else:
        # create new nodes using the actions defined
        next_nodes = []
        # Creating a list of angles to be checked
        check_angles = [-60, -30, 0, 30, 60]
        # Moving along each angle
        for a in check_angles:
            empty,next_node = action(active_node,a)
            next_nodes.append([empty,next_node])
        
        # Calculating the cost to move to new nodes
        for n in next_nodes:
            if (n[0] == True) and (n[1] not in closednode_list):
                cv2.line(canvas,(active_node[0],active_node[1]),(n[1][0],n[1][1]),white, 1)
                if n[1] not in opennode_list:
                    # Adding the node to opennode list
                    opennode_list[n[1]] = [c2c+M, wt*C2G(n[1]),active_node]
                else:
                    # Calculating the total cost
                    if opennode_list[n[1]][0] + opennode_list[n[1]][1] > c2c+ M + wt*C2G(n[1]):
                        opennode_list[n[1]] = [c2c+M, wt*C2G(n[1]), active_node]
    # Print start and goal nodes on canvas
    cv2.circle(canvas, (start_node[0],start_node[1]), 5, green, -1)
    cv2.circle(canvas, (goal_node[0], goal_node[1]), 5, green, -1)
    canvas = cv2.flip(canvas,0)
    cv2.imshow('Output',canvas)
    cv2.waitKey(1) 
    canvas = cv2.flip(canvas,0)
else:
    # If the goal is unreachable print error code
    print("The goal cannot be reached, Please try again!!")
    
end = time.time()
print(f"Total time taken is {end - begin}")

