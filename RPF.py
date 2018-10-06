#the following function alters the position of the node
#depending on its proximity to obstacles
def repulsionForce(coor,obs):
    #repulsion constant
    K = 10
    #distance constant
    C = 2
    #check each obstacle
    for obj in obs:
        #get the position of each obstacle
        pos = obj.getPoint()
        #check the distance from the obstacle
        distance = np.sqrt((pos[0]-coor[0])**2+(pos[1]-coor[1])**2)
        #apply function if the distant is less than C units away. 
        if distance < C:
            #modify the position along a vector
            angle = np.arctan2((coor[1]-pos[1]),(coor[0]-pos[0]))
            newdistance = (1/(K*distance))
            newX = newdistance*np.cos(angle) + coor[0]
            newY = newdistance*np.sin(angle) + coor[1]
            coor[0] = newX
            coor[1] = newY
    return coor

#The following function is used for path planning to detect whether a node is
#within the range of an obstace, or moving out of the bounds of the c-space.
def checkCollision(point,obstacles):
    #obstacles are of stem class
    #go through each obstacle
    for obstacle in obstacles:
        #if any of the conditions below are true, True is returned to notify
        #other functions that the node in question is colliding.
        #check if point is within obstacle
        if obstacle.isInside(point[0],point[1]):
            return True
        #check if node is outside of bounds
        if point[0] > 6 or point[0] < -6:
            return True
        if point[1] > 4 or point[1] < -4:
            return True
    #if true was never triggered while searching through all points, return
    #false. Collision does not exist.
    return False

#The following function forms a path 
def makePath(graph,goal):
    #iterate backwards throught the graph to find the goal node.
    #This is performed backwards, simply because it is more likely
    #the goal node is towards the end of the graph.
    for itr in reversed(range(0,len(graph))):
        #Check if the point in the graph is the goal that we 
        #are looking for
        if graph[itr].getPoint() == goal.getPoint():
            #save the index of this point if it is
            Gindex = itr
            #get out of the loop
            break
    #initialize list to hold path    
    path = []
    #set the focused node to the goal node
    currNode = graph[Gindex]
    #move backwards following the parents of the nodes to find the path
    while(True):
        #add points of node to path list
        path.append(currNode.getPoint())
        #if the current node has no parents, then the source has been reached
        if currNode.getParents() == None:
            break    
        #set the next node to be the parent of the current node
        currNode = currNode.getParents()
    #The path is found backwards. So reverse the list and then return it.    
    return path[::-1]

#The following function is used by the best-first portion of the random
#potential field algorithm. The VSM is defined by 8 possible positions
#the path can travel to: North, northeast, east, southeast, south, southwest, west, northwest.
#LPM is defined by how one of these positions are chosen. The position that recudes the distance
#to the destination the most is the chosen position.
def BFS(start,end,obs):
    #declare a step variable size. This is the size of the square region
    #the algorithm will search for positions to move to.
    STEP = 0.2
    #Retrieve the start and end positions
    startCoor = start.getPoint()
    endCoor = end.getPoint()
    #initialize an empty path. This will be a list of all the generated nodes
    Path = []
    #keep this loop running until it is broken from the interior
    while(True):
        #Retrieve the parent node, if no nodes have been added to path, then this
        #is the start node
        if len(Path) == 0:
            #set parent node to start node
            closest = start
        else:
            #set parent node, to most recently added node
            closest = Path[-1]
        #get the position of this point
        cPoint = closest.getPoint()
        #determine the distance between the goal node and the parent node
        distance = np.sqrt((cPoint[0]-endCoor[0])**2+(cPoint[1]-endCoor[1])**2)
        #if the distance between these two nodes, is less than the step size,
        #then the goal node has essentially been reached.
        if distance < STEP:
            #create a new node out of the goal node
            tempN = nd.node(endCoor)
            #if the path is not size 0 then create a two way connection
            #between the parent and the child
            if len(Path) != 0:
                Path[-1].addChild(tempN)
                tempN.addParent(Path[-1])
            Path.append(tempN)
            #once the goal is reached, we can break
            break
        #if the goal has not been reached prepare to find a new node to jump to
        #retrieve the position of the parent node
        xs = cPoint[0]
        ys = cPoint[1]
        #the array below represent all the possible grid positions the parent
        #can move to.
        children = [[xs,ys+STEP],[xs+STEP,ys],[xs,ys-STEP],[xs-STEP,ys],
                    [xs+STEP,ys+STEP],[xs+STEP,ys-STEP],[xs-STEP,ys-STEP], 
                    [xs-STEP,ys+STEP]]
        #initialize the position of the next point to none
        nextCoor = None
        #go through the list of possible adjacent positions to move to, and
        #determine the most lucrative direction. This is determined, by the 
        #direction that will shorten the path the most, while avoiding obstacles
        for child in children:
            #find the distance from the goal if one were to move to this node
            projDist = np.sqrt((child[0]-endCoor[0])**2+(child[1]-endCoor[1])**2)        
            #if the distance is smaller than the current distance AND avoids 
            #obstacles, then save this node as a possible candidate
            if projDist < distance and not checkCollision(child,obs):
                distance = projDist
                nextCoor = child
        #if a node was found add it to the chain of previously found nodes
        if nextCoor != None:
            #alter the position of this node by passing it through a 
            #replusion function
            nextCoor = repulsionForce(nextCoor,obs)
            #create a new node from the new coordinates
            newNode = nd.node(nextCoor)
            if len(Path) != 0:
                #create parent-child relationship 
                Path[-1].addChild(newNode)
                newNode.addParent(Path[-1])
            #add node to the chain of nodes
            Path.append(newNode)
        else: #local minima has been reached
            break
    #return the generated path
    return Path
        
#The following is a random walk function. This function is triggered when the 
#BFS function in the Random Potential Fields algorithm fails to reach the goal
def RW(start,end,obs):
    #Set a constant number of random walks to be preformed before stopping
    K = 20
    #declare step size
    STEP = 0.2
    #get staring point coordinates
    startCoor = start.getPoint()
    xs = startCoor[0]
    ys = startCoor[1]
    #Declare possible positions to move to
    children = [[xs,ys+STEP],[xs+STEP,ys],[xs,ys-STEP],[xs-STEP,ys], 
                [xs+STEP,ys+STEP],[xs+STEP,ys-STEP],[xs-STEP,ys-STEP], 
                [xs-STEP,ys+STEP]]
    #initialize empty path list
    Path = []
    #loop K times to generate K random new nodes in hope of getting out of a
    #local minimum
    for itr in range(0,K):
        #Assume the the selected node is in a collision area
        blocked = True
        while blocked:            
            #select a random direction
            sct = ran.randint(0,7)
            #retrieve that direction
            sctDirc = children[sct]
            #check if this direction is in an obstacle
            blocked = checkCollision(sctDirc,obs)
        #create a new node out of the randomly generated node        
        tempn = nd.node(sctDirc)
        #if this is the first node, append it to the empty path list
        if len(Path) == 0:
            Path.append(tempn)
        # if this is not the first node, create a parent-child relationship
        #with the previous node
        else:
            Path[-1].addChild(tempn)
            tempn.addParent(Path[-1])
            Path.append(tempn)
        #reset startCoor so that it now points to the newest node. 
        startCoor = tempn.getPoint()
        #similarly, update the xs,ys, and children variables
        xs = startCoor[0]
        ys = startCoor[1]
        children = [[xs,ys+STEP],[xs+STEP,ys],[xs,ys-STEP],[xs-STEP,ys], 
                    [xs+STEP,ys+STEP],[xs+STEP,ys-STEP],[xs-STEP,ys-STEP], 
                    [xs-STEP,ys+STEP]]        
    return Path
        
#The following is the main function for the random potentional fields algorithm.
#The algorithm takes a start position, an end position, and the present
#obstacles. It then switches back and fourth from BFS and random walk functions
#to reach the end point.
def RandomPotentialFields(start,end,obs):
    #Get the end point coordiantes
    endCoor = end.getPoint()
    #A will be the final list of nodes that lead to the goal
    A = [start]
    #Initialize Randomwalk to be true
    RandomWalk = True
    #keep this function running until the goal is reached
    while(True):
        #check if the most recently added point to the path is the end goal.
        #if it is break out of the loop, the path is complete.
        if A[-1].getPoint() == endCoor:
            break
        #otherwise, start generating path
        else:
            #set the new path to an empty list.
            path = []
            #if we previously randomwalked, then we now want to use BFS
            if RandomWalk:
                #set randomwalk to false 
                RandomWalk = False
                #generate a new path using BFS
                path = BFS(A[-1],end,obs)
            #if we previously BFS'd, then we now want to random walk
            else:
                #set randomwalk to true and
                RandomWalk = True
                #generate a new path using the random walk
                path = RW(A[-1],end,obs)
        #if the length of the path is zero, break. You are in trouble
        if len(path) == 0:
            break
        #other wise create parent-child relationship between the last added
        #node in A and the first added node in the the newly generated path.
        A[-1].addChild(path[0])
        path[0].addParent(A[-1])
        #concatenate these two lists
        A = A + path
    #when the goal has been reached, return the completed path
    return A

#There are often multiple solutions when solving inverse kinematics. However,
#we want our solution to minimize the angle change between the next and previous
#arm positions. This function selects the series of angles with the smallest 
#magnitude difference compared to the previous arm orientation.
def MinimizeAngle(prevAng,newAngles):
    #set this to super large number
    mag = 10000
    best = None
    #iterate through all the found angles
    for angles in newAngles:
        #compute the difference of each angle change
        diff = [prevAng[0]-angles[0],prevAng[1]-angles[1],prevAng[2]-angles[2]]
        #find the magintude of this difference
        pwr = np.sqrt(diff[0]**2 + diff[1]**2 + diff[2]**2)
        #if this magnitude is smaller than the previous largest value, replace it
        if pwr < mag:
            mag = pwr
            best = angles
    #return the best angle option
    return best

#The following function takes a series of angles that represent joint angles of 
#the robot, the robot, and all the obstacles. The function returns True if a 
#collision between a robot arm and a stem has been detected and False if no
#collisions occur.
def isArmCollision(angle,robot,obs):
    #get the position of the joints
    positions = robot.getJointPos(angle)
    #go through each pair of joints and compare them against
    #every obstace that they could possible hit.
    for itr in range(0,len(positions)):
        if itr == len(positions)-1:
            break
        #retrieve the joint positions and convert them to float lists
        j1 = positions[itr]
        j1 = [float(j1[0]),float(j1[1])]
        #There are two joints representing the start and end of a 
        #segment
        j2 = positions[itr+1]
        j2 = [float(j2[0]),float(j2[1])]
        #go through each stem and check if collision has occured
        for obj in obs:
            #get the stems position
            objpos = obj.getPoint()
            #Condition if the slope of the segment is undefined
            if (j1[0]-j2[0]) == 0:
                #see if the line is within range to intersect
                test1 = np.sqrt((j1[0]-objpos[0])**2) < obj.getRad()
                #see if either the ends of the segment intersect the stem
                test2 = obj.isInside(j1[0],j1[1])
                test3 = obj.isInside(j2[0],j2[1])
                #check if the stem is between both the end points of the segment
                test4 = (j1[1] < objpos[1] and j2[1] >objpos[1]) or (j1[1]>objpos[1] and j2[1]<objpos[1])
                # if the segment is within range and is inside the circle, then there is collision
                if (test1) and (test2 or test3 or test4):
                    return True
            #The following is similar as the undefined case, but instead checks for
            #the zero slope case
            elif (j1[1]-j2[1]) == 0:
                test1 = np.sqrt((j1[1]-objpos[1])**2) < obj.getRad()
                test2 = obj.isInside(j1[0],j1[1])
                test3 = obj.isInside(j2[0],j2[1])
                test4 = (j1[0] < objpos[0] and j2[0] >objpos[0]) or (j1[0]>objpos[0] and j2[0]<objpos[0])
                if (test1) and (test2 or test3 or test4):
                    return True 
            #Otherwise, we must check for the case that the segment has a slope 0 < x < inf
            #this is done through finding the distance of the segment to the obstace.
            #A perpendicular line to the robots arm that intersects the center of the circle
            #is used to determine the distance. Then similar checks to the previous two
            #cases are preformed to find collision
            else:
                #get slope of the robotic arm segment
                slope = (j1[1]-j2[1])/(j1[0]-j2[0])
                #get b intercept 
                b1 = j1[1] - slope*j1[0]
                #get perpendicular slope of robotic arm
                pslope = -1/slope
                #determine b intercept of the perpendicular line that intersects
                #the circle center
                bp = objpos[1] - pslope*objpos[0]
                #solve for the x,y on the robotic arm segment that intersects with
                #the perpendicular line
                X = (bp-b1)/(slope-pslope)
                Y = pslope*X +bp
                #check for collision
                test1 = np.sqrt((objpos[0]-X)**2+(objpos[1]-Y)**2) < obj.getRad()
                test2 = obj.isInside(j1[0],j1[1])
                test3 = obj.isInside(j2[0],j2[1])
                test4 = ((j1[0]<objpos[0] and j1[1]<objpos[1] and j2[0]>objpos[0] and j2[1]>objpos[1]) or (j1[0]>objpos[0] and j1[1]>objpos[1]and j2[0]<objpos[0] and j2[1]<objpos[1]) or (j1[0]<objpos[0]and j1[1]>objpos[1] and j2[0]>objpos[0] and j2[1]<objpos[1]) or (j2[0]<objpos[0] and j2[1]>objpos[1] and j1[0]>objpos[0] and j1[1]<objpos[1]))
                if test1 and (test2 or test3 or test4):
                    return True    
    #if collision is never detected, return False
    return False

#This function is given a path, the robot, the series of obstacles. The 
#function goes through each node and determines the angles for the robots
#joints that allow it to fit in that position. If no joints are found for
#a given angle, then the angle increases and is checked again. This is done
# until each node has an associated set of angles
def RobotMotion2(path,robot,obs):
    #Set an array of all the angles to empty
    Q = []
    #get the end path
    endPoint = path[-1]
    #check if the position of the destination is too far away. If it is
    #then there is no point of checking.
    disp = np.sqrt((endPoint[0])**2+(endPoint[1])**2)
    if disp > 4.5:
        print " location is too far away"
        return False
    #go through each node in the path
    for node in path:
        #get the position of the node
        pos = node
        qs = []
        #create an arbitrary starting angle
        theta = 0
        #record this angle in permanent variable
        ogtheta = theta
        #assume collision starts of as true
        collision = True
        while collision:
            #check if theta as gone past 2pi. if it has
            #then all the angles hace been checked and we can break.
            if theta > 2*np.pi:
                break
            #Find rotation matrix
            R0T = bot.rot(theta)
            r11 = float(R0T[0,0])
            r12 = float(R0T[0,1])
            r21 = float(R0T[1,0])
            r22 = float(R0T[1,1])  
            M = np.matrix([[r11, r12, pos[0]],[r21, r22, pos[1]],[0, 0, 1]])
            #find the angles from the inverse kinematics
            qs = robot.invKin(M)
            #set the solution to none
            tempSol = None
            #go through each angle that was determined through inverse kinematics
            for ang in qs:
                #check if the arm is in a collision region
                if isArmCollision(ang,robot,obs):
                    #if it set collision to true, this will force the loop 
                    #to continue
                    collision = True
                else:
                    #otherwise, set collision to false, assign a solution
                    # and break
                    collision = False
                    tempSol = ang
                    break
            #if there is a solution assign it to the list of all angles
            #and continue on to the next node in the path by breaking
            if tempSol != None:
                Q.append(tempSol)
                break
            theta += np.pi/180
    #return all the angles
    return Q

if __name__ == "__main__":
    #import libraries and classes
    import random as ran
    import numpy as np
    import node as nd
    import math as m
    import bot
    import berry as ber
    import stem as st
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import timeit
    
    numCollected = []
    timetaken = []
    COUNTER = 0
    while COUNTER < 1:
        berries = [] #list to hold berries
        stems = [] #list to hold stems
        robot1 = bot.Bot(2.0,1.5,1.0) #robot object
        M = np.matrix([[1,0,1],[0,1,3.5],[0, 0, 1]])
        Q = robot1.invKin(M)
       
        #for loop creates stem objects
        for i in range(0,10):
            #pick a random y coordinate
            y =  round(ran.uniform(-1,1) * 4,2)
            #pick a random x coordinate
            x =  round(ran.uniform(-1,1) * 6,2) 
            #If the stem sits on the axis move it away by 1 unit
            if x < 1 and x > -1:
                if x > 0:
                    x += 1
                if x < 0 :
                    x -= 1
            if y < 1 and y > -1:
                if y > 0:
                    y += 1
                if y < 0 :
                    y -= 1
            #create stem object
            stem = st.Stem(x,y,0.5)
            #place stem with other stem objects
            stems.append(stem) 
        #for loop creates berry objects
        for i in range(0,20):
            #pick a random y coordinate
            while(True):
                y =  round(ran.uniform(-1,1) * 4,2) 
                #pick a random x coordinate
                x =  round(ran.uniform(-1,1) * 6,2)
                #create berry object
                if not checkCollision([x,y],stems):
                    break
            berry = ber.Berry(x,y)
            #place berry in a list of other berry objects
            berries.append(berry)   
            
        ############################plot
        #Set the axis    
        fig, ax = plt.subplots()
        plt.axis([-6, 6, -4, 4])
        #draw berries
        for berry in berries:
            pos = berry.getPoint()
            plt.plot(pos[0], pos[1], 'ro') 
        #draw stems
        for stem in stems:
            pos = stem.getPoint()
            circ = plt.Circle((pos[0], pos[1]),radius = 0.5,color="black")
            ax.add_artist(circ)
        #plt.pause(5)    
        #Starting position 
        begin = [4.5,0]
        Success = 0
        Failures = 0
        plt.pause(8)
        for berry in berries:
            ##################################################
            #The Following implements Random Potential Fields#
            ##################################################
            startTime = timeit.default_timer()
            berryPos = berry.getPoint()
            tempN = nd.node(begin)
            print "goal: " + str(berryPos)
            print "part 1"
            rpf = RandomPotentialFields(tempN,berry,stems)
            print "Generated " + str(len(rpf)) + " points."
            print "part 2"
            path = makePath(rpf,rpf[-1])
            print "path generated to get to berry"
            x = []
            y = []
                
            Q = RobotMotion2(path,robot1,stems)
            endTime = timeit.default_timer()
            timetaken.append(startTime-endTime)
            print "all angles were determined to get to berry"
            if Q == False: # or len(Q) != len(path)''':
                #Failed to make path or reach berry
                Failures += 1
                continue
            drawnPath = []
            for p in range(0,len(path)):
                if p == len(path)-1:
                    continue
                x = [path[p][0],path[p+1][0]]
                y = [path[p][1],path[p+1][1]]
                green,= plt.plot(x, y, 'go-')
                drawnPath.append(green)
                
            pos = berries[0].getPoint()
            plt.plot(pos[0],pos[1],'ro')
            rect = patches.Rectangle((.875,-.125),.25,.25,linewidth=1,edgecolor='m',facecolor='none')
            # Add the patch to the Axes
            ax.add_patch(rect)
            #draw the robot
            if Q != False:
                for Aset in Q:
                    pos = robot1.getJointPos(Aset)
                    p1 = pos[0]
                    p2 = pos[1]
                    p3 = pos[2]
                    x = [0,float(p1[0]),float(p2[0]),float(p3[0])]
                    y = [0,float(p1[1]),float(p2[1]),float(p3[1])]
                    ln, = plt.plot(x,y,'bo-') 
                    plt.pause(0.005)
                    ln.remove()
            for itr in drawnPath:
                itr.remove()
            ##########################################Part2
            startTime = timeit.default_timer()
            tempN2 = nd.node([1,0])
            #move the berry to the basket
            begin = berryPos
            tempN = nd.node(begin)
            tempN2 = nd.node([1,0])
            print "goal: " + str([1,0])
            print "part 1"
            rpf = RandomPotentialFields(tempN,tempN2,stems)
            print "Generated " + str(len(rpf)) + " points."
            print "part 2"
            path = makePath(rpf,rpf[-1])
            print "path generated to get home"
            x = []
            y = []
            
            Q = RobotMotion2(path,robot1,stems)
            endTime = timeit.default_timer()
            timetaken.append(startTime-endTime)
            print "all angles were determined to get to home"
            #The following is for drawing the final path
            drawnPath = []
            for p in range(0,len(path)):
                if p == len(path)-1:
                    continue
                x = [path[p][0],path[p+1][0]]
                y = [path[p][1],path[p+1][1]]
                green,= plt.plot(x, y, 'go-')
                drawnPath.append(green)
    
                
            pos = berries[0].getPoint()
            plt.plot(pos[0],pos[1],'ro')
            rect = patches.Rectangle((.875,-.125),.25,.25,linewidth=1,edgecolor='m',facecolor='none')
            # Add the patch to the Axes
            ax.add_patch(rect)
            #draw the robot
            if Q != False:
                for Aset in Q:
                    pos = robot1.getJointPos(Aset)
                    p1 = pos[0]
                    p2 = pos[1]
                    p3 = pos[2]
                    x = [0,float(p1[0]),float(p2[0]),float(p3[0])]
                    y = [0,float(p1[1]),float(p2[1]),float(p3[1])]
                    ln, = plt.plot(x,y,'bo-') 
                    plt.pause(0.005)
                    ln.remove() 
            begin = [1,0]
            #remove green path
            for itr in drawnPath:
                itr.remove() 
            print "completed collecting this berry"
            Success +=1
        numCollected.append(Success)
        
        print "Finished"
        print "Robot was able to collect " + str(Success) + " berries."
        print "Robot failed to collect " + str(Failures) + " berries."
        COUNTER +=1
    
    print "Number of berries collected in each random plotting:"
    print numCollected
    print "Length of time taken to plan each path"
    print timetaken
    print "Average number of berries collected"
    print sum(numCollected)/len(numCollected)
    print "Average time taken to make path"
    print sum(timetaken)/len(timetaken)
    
    plt.pause(3)
    print "Closing"    
    
    