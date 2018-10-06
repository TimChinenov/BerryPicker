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

#This function finds the closest point to p. This is the VERTEX SELECTION 
#METHOD for the RDT algorithm. After a random node is generated, it is connected
#to the closest existing node.
def closestPoint(p,points):
    #set the distance value to something absurdly large
    smallVal = 100000
    #set the first value as the smallest value
    smallest = points[0]
    #get the coordinates of the point in question
    pcoor = p.getPoint()
    #go through each coordinate in points and compare
    for point in points:
        #get the position of the current iterated point
        pos = point.getPoint()
        #find the distance between the iterated point and the point in question
        distance = np.sqrt((pcoor[0]-pos[0])**2+(pcoor[1]-pos[1])**2)
        #if the distance is smaller than the current smallval then
        #set this as the new closest node
        if distance < smallVal:
            smallVal = distance
            smallest = point
    #after iterating through all the points return
    return smallest

#This function takes in two node objects that represent the starting point and
#the goal point. The object than takes a step parameter that determines the 
#distance the function should take between each node. Finally, a list
#of objects is passed. The function will make a linear chain of nodes from 
#start to end. For each new node created, it will check whether the node 
#is inside and obstacle.
def chainPoints(start,end,step,obs):
    #This is the final list of nodes that will be returned
    pointsList = []
    #get coordinates of the starting position
    startCoor = start.getPoint()
    #get coordinates of the ending position
    endCoor = end.getPoint()
    #make sure that the coordinates do not have an underfined slope
    if (endCoor[0]-startCoor[0]) != 0:
        #find the slope between the end and start
        slope = (endCoor[1]-startCoor[1])/(endCoor[0]-startCoor[0])
        #determine the angle of the slope
        theta = np.arctan(slope)
    else:
        #if the slope is undefined, set it to none. A seperate series of
        #conditionals will handle and undefined slope.
        slope = None
    #find the distance in order to figure out how many points need to be made.
    distance = np.sqrt((startCoor[0]-endCoor[0])**2+(startCoor[1]-endCoor[1])**2)
    #figure out out how many new points need to be generated.
    num = int(np.floor(distance/step))
    #The constants below are used to change the direction of the generated points
    #although we know the slope, the points need to be specifically generated in
    #the direction of the vector.
    dircX = 1
    dircY = 1
    #The below alters the dirc constants, if the vector is in one of the specific
    #orientations
    if startCoor[1] > endCoor[1] and startCoor[0] > endCoor[0]:
        dircY = -1
        dircX = -1
    if startCoor[0] > endCoor[0] and startCoor[1] < endCoor[1]:
        dircX = -1
        dircY = -1
    #The following loop iterates through the number of required new nodes to 
    #generate. Each iteration, it will determine the position of the new node
    #and whether or not it can in fact be placed. 
    for itr in range(1,num):
        #if the slope is defined, use the following equation to determine the 
        #position of the next node.
        if slope != None:
            nX = startCoor[0] + np.cos(theta)*step*itr*dircX
            nY = startCoor[1] + np.sin(theta)*step*itr*dircY
        #if the slope is undefined, then move in the positive or negative y 
        #direction    
        else:
            nX = startCoor[0]
            nY = startCoor[1] + step*itr*dircY
        #Take the generated coordinates and check if they collide with any
        #obstacles
        if checkCollision([nX,nY],obs):
            #if a collision does indeed occur, end the function and return the chain
            #up to the point that has been reached.
            return pointsList
        #create a temporary new node
        tempn = nd.node([nX,nY])
        #if a chain has been created, form a two way connection between the parent
        #and the child and add the child to the total chain list
        if (len(pointsList) > 0):   
            pointsList[itr-2].addChild(tempn)
            tempn.addParent(pointsList[itr-2])
        pointsList.append(tempn)
    #Once the completed chain has been formed, add the ending node to the chain.
    if len(pointsList) != 0:    
        pointsList[-1].addChild(end)
        end.addParent(pointsList[-1])
        pointsList.append(end)
    #if the chain is empty, simply add the ending node to the chain.    
    else:
        pointsList.append(end)
    #return the new chain of points
    return pointsList        

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

#This will be the main rapidly expanding dense trees function
#q0 is the starting point and qg is the goal. Region determines how large of a region
#the tree needs to be in to determine whether it has reached a goal
def RDT(q0,qg,region,obstacles):
    #The step size taken between nodes.
    STEP_SIZE = .2
    #Create a node out of the start node
    start = nd.node(q0)
    if np.sqrt((q0[0]-qg[0])**2+(q0[1]-qg[1])**2) >4.5:
        return False
    #declare the region that suffices as the goal (cardinal directions)
    xMaxGoal = qg[0]+region
    xMinGoal = qg[0]-region
    yMaxGoal = qg[1]+region
    yMinGoal = qg[1]+-region
    #Boolean changes once the goal has been reached
    GoalReached = False
    #the following contains a list of new points. The new points will be generated as the tree
    #expands.
    newPoints = [start]
    #This list contains the tree of ALL generated points
    Points = [start]    
    #While the goal has not been reached, stay in this while loop
    while GoalReached == False:
        #The following for loop checks to make sure that the goal has not been reached.
        #it compares the coordinates of the newly generated points with the goal point. This
        #saves us the trouble from checking points multiple times.
        for point in newPoints:
            curCoor = point.getPoint() #get the coordinates of the current point
            #check if this point is in the region of the goal. 
            if curCoor[0] < xMaxGoal and curCoor[0] > xMinGoal and curCoor[1] < yMaxGoal and curCoor[1] > yMinGoal:
                #if region has been entered set goal reached to true.
                GoalReached = True
                #set the point that triggered this event to a variable for safekeeping.
                goalPoint = point
                endn = nd.node([qg[0],qg[1]])
                goalPoint.addChild(endn)
                endn.addParent(goalPoint)
                Points.append(endn)
                goalPoint = endn
                #leave this for loop no reason to stay in it.
                return (Points,goalPoint)
        #set blocked as true by default    
        blocked = True
        while(blocked):
            #pick a random y coordinate 
            y =  round(ran.uniform(-1,1) * 4,2) 
            #pick a random x coordinate
            x =  round(ran.uniform(-1,1) * 6,2)
            gpoint = [x,y]
            #check if the generated point collides with anything, if it
            #doesnt, the while loop will be left
            blocked = checkCollision(gpoint,obstacles)
        #create a new node for the newly generated point
        nextPoint = nd.node(gpoint)
        #finding the closest node to the randomly generated node. 
        neighbor = closestPoint(nextPoint,Points)
        #Create a chain of nodes from the neighbor to the next point 
        connection = chainPoints(neighbor,nextPoint,STEP_SIZE,obstacles)
        #If the chain has a length of zero then add the nextPoint directly
        #to the neighbor
        if len(connection) == 0:
            continue
            #neighbor.addChild(nextPoint)
            #nextPoint.addParent(neighbor)
            #Points.append(nextPoint)
            #newPoints = [nextPoint]
        #otherwise, incorperate the new chain into the list of all points
        else:
            #add this chain to the child of the nearest found point
            neighbor.addChild(connection[0])
            #add the reverse relationship as well
            connection[0].addParent(neighbor)
            #add this new set of points to the existing points
            Points = Points + connection
            newPoints = connection
    return

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

def RobotMotion2(path,robot,obs):
    Q = []
    endPoint = path[-1]
    disp = np.sqrt((endPoint[0])**2+(endPoint[1])**2)
    if disp > 4.5:
        print " location is too far away"
        return False
    for node in path:
        pos = node
        qs = []
        #create an arbitrary starting angle
        theta = 0
        #record this angle in permanent variable
        ogtheta = theta
        collision = True
        while collision:
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
            tempSol = None
            for ang in qs:
                if isArmCollision(ang,robot,obs):
                    collision = True
                else:
                    collision = False
                    tempSol = ang
                    break
            if tempSol != None:
                Q.append(tempSol)
                break
            theta += np.pi/180
    return Q
                
            
    
#This function is given a path and determines a series of joint angles 
#that can be followed to move the robot to its destination 
def RobotMotion(path,robot,obs):
    #The following will be an array that contains a list of angle positions for each node.
    Q = []
    #Get the end point node
    endPoint = path[-1]
    #Check the displacement from the origin
    disp = np.sqrt((endPoint[0])**2+(endPoint[1])**2)
    #if the displacement is greater than 4.5 (the length of the arm), then 
    #the berry is unreachable.
    if disp > 4.5:
        print "Berry is too far away"
        return False
    #go through each node in the path and determine inverse kinematic angles
    #such that angle transitions are minimized
    for node in path:
        #get the point of the node of interest
        pos = node
        #create a new empty set of angles
        qs = []
        #create an arbitrary starting angle
        theta = 5*np.pi/4
        #record this angle in permanent variable
        ogtheta = theta
        collision = False
        while len(qs) == 0 and not collision:
            if theta >= 2*np.pi:
                theta = theta - 2*np.pi 
            #Find rotation matrix
            R0T = bot.rot(theta)
            r11 = float(R0T[0,0])
            r12 = float(R0T[0,1])
            r21 = float(R0T[1,0])
            r22 = float(R0T[1,1])
            #Declare end point matrix
            M = np.matrix([[r11, r12, pos[0]],[r21, r22, pos[1]],[0, 0, 1]])
            #find the angles from the inverse kinematics
            qs = robot.invKin(M)
            #assuming that angles were found, find the best angle
            if len(qs) != 0 and len(Q) != 0:
                best = MinimizeAngle(Q[-1],qs)
                #Check if the arm is colliding with anything
                if isArmCollision(best,robot,obs):
                    #increment theta and set collison to true
                    print "collision has occured"
                    collision = True
                    theta += np.pi/180
                    continue
                #if collision didn't occur, append new acceptable angles
                else:
                    Q.append(best)
                    collision = False
                    break
            elif len(qs) != 0 and len(Q) == 0:
                Q.append(qs[0])
                break
            theta += np.pi/180
            #if every angle has been tested, break
            if round(theta,2) == round(ogtheta,2):
                break
    return Q

if __name__ == "__main__":
    #import libraries and classes
    import point as pt
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
    while COUNTER < 50:
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
            
        #stems = []
        ###########################plot
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
        rect = patches.Rectangle((.875,-.125),.25,.25,linewidth=1,edgecolor='m',facecolor='none')
        # Add the patch to the Axes
        ax.add_patch(rect)        
        #Starting position 
        begin = [4.5,0]
        Success = 0
        Failures = 0
        
        for berry in berries:
            startTime = timeit.default_timer()
            berryPos = berry.getPoint() 
            ################################
            #The following implements RDT's#
            ################################        
            tempN = nd.node(begin)
            print "goal: " + str(berryPos)
            print "part 1"
            rdt = RDT(tempN.getPoint(),berryPos,0.2,stems)
            if rdt == False:
                print "returned false"
                continue
            print "Generated " + str(len(rdt[0])) + " points."
            print "part 2"
            path = makePath(rdt[0],rdt[1])
            x = []
            y = []
                
            Q = RobotMotion2(path,robot1,stems)
            endTime = timeit.default_timer()
            timetaken.append(endTime-startTime)
            print "all angles were determined to get to berry"
            if Q == False  or len(Q) != len(path):
                ##Failed to make path or reach berry
                Failures += 1
                continue
            #The following is for drawing RDT's   
            '''
            for gr in rdt[0]:
                if len(gr.getChildren()) == 0:
                    pos = gr.getPoint()
                    plt.plot(pos[0],pos[1],'yo')
                else:
                    pos = gr.getPoint()
                    for child in gr.getChildren():
                        childPos = child.getPoint()
                        x = [pos[0],childPos[0]]
                        y = [pos[1],childPos[1]]
                        plt.plot(x,y,'yo-')
            #'''
            drawnPath = []
            for p in range(0,len(path)):
                if p == len(path)-1:
                    continue
                x = [path[p][0],path[p+1][0]]
                y = [path[p][1],path[p+1][1]]
                green,= plt.plot(x, y, 'go-')
                drawnPath.append(green)
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
            ################################
            #The following implements RDT's#
            ################################  
            tempN = nd.node(berryPos)
            tempN2 = nd.node([1,0])
            print "goal: " + str(tempN2.getPoint())
            print "part 1"
            rdt = RDT(tempN.getPoint(),tempN2.getPoint(),0.2,stems)
            if rdt == False:
                continue            
            print "Generated " + str(len(rdt[0])) + " points."
            print "part 2"
            path = makePath(rdt[0],rdt[1])
            x = []
            y = []
            
            Q = RobotMotion2(path,robot1,stems)
            endTime = timeit.default_timer()
            timetaken.append(endTime-startTime)
            #The following is for drawing RDT's  
            '''
            for gr in rdt[0]:
                if len(gr.getChildren()) == 0:
                    pos = gr.getPoint()
                    plt.plot(pos[0],pos[1],'yo')
                else:
                    pos = gr.getPoint()
                    for child in gr.getChildren():
                        childPos = child.getPoint()
                        x = [pos[0],childPos[0]]
                        y = [pos[1],childPos[1]]
                        plt.plot(x,y,'yo-')
            '''
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