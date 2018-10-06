import numpy as n
def rot(theta):
    #Function performs a 2-D matrix rotation based on theta
    R = n.matrix([[n.cos(theta), (-1)*n.sin(theta)],
                    [n.sin(theta), n.cos(theta)]])
    return R

def eye(val):
    #function creates val x val identity matrix
    ident = []
    for i in range(0,val):
        tempI = [0]*val
        tempI[i] = 1
        ident.append(tempI)
    identM = n.asarray(ident)
    return identM

#The following is the robot class
class Bot:
    def __init__(self,l1,l2,l3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.p01 = n.matrix([[l1],[0]])
        self.p12 = n.matrix([[l2],[0]])
        self.p23 = n.matrix([[l3],[0]])
        
    #The following solves the inverse kinematics for the robot
    #using expressions determined through DH parameter analysis
    def invKin(self,M):
        p0T = M[0:2,2]
        r0T = M[0:2,0:2]
        angle = n.arcsin(r0T[1,0])
        px = float(p0T[0])
        py = float(p0T[1])
        ########################################
        #solve the the position of the second angle
        th2 = []
        a = ((px - self.l3*float(r0T[0,0]))**2 + (py - self.l3*float(r0T[1,0]))**2 - self.l1**2 - self.l2**2)/(2.0*self.l1*self.l2)
        if not (a**2 > 1):
            temp = n.arctan2((1-a**2)**(0.5),a)
            th2.append(temp)
            if temp != 0:
                th2.append(n.arctan2((-1)*(1-a**2)**(0.5),a))
        ########################################
        #solve for the angle of the end effector
        Q = []
        for theta in th2:
            c = (px**2)+(py**2)-(self.l1**2)-(self.l2**2)-(self.l3**2)-(2*self.l1*self.l2*n.cos(theta))
            a = 2*self.l2*self.l3+2*self.l1*self.l3*n.cos(theta)
            b = -2*self.l1*self.l3*n.sin(theta)
            ans = n.arctan2(b,a)+n.arctan2((a**2+b**2-c**2)**0.5,c)
            Q.append([theta,ans])
            ans = n.arctan2(b,a)-n.arctan2((a**2+b**2-c**2)**0.5,c)
            Q.append([theta,ans])
        ########################################
        #solve for the first angle
        nQ = []
        for qs in Q:
            theta2 = qs[0]
            theta3 = qs[1]
            theta1 = angle - theta2 - theta3
            if not (n.isnan(theta1) or n.isnan(theta2) or n.isnan(theta3)):
                nQ.append([theta1,theta2,theta3])
        #double check solutions to make sure the match up with fwdkin
        #if the do not, throw them out.
        Q = []
        for qs in nQ:
            ang = self.fwdKin(qs)
            r11 = round(float(ang[0,0]),2)
            r12 = round(float(ang[0,1]),2)
            r13 = round(float(ang[0,2]),2)
            r21 = round(float(ang[1,0]),2)
            r22 = round(float(ang[1,1]),2)
            r23 = round(float(ang[1,2]),2)  
            M11 = round(float(M[0,0]),2)
            M12 = round(float(M[0,1]),2)
            M13 = round(float(M[0,2]),2)
            M21 = round(float(M[1,0]),2)
            M22 = round(float(M[1,1]),2)
            M23 = round(float(M[1,2]),2)
            if r11==M11 and r12==M12 and r13==M13 and r21==M21 and r22==M22 and r23==M23:
                Q.append(qs)
        return Q
    
    #The following function determines the end effector of the robot 
    #based on the given angles
    def fwdKin(self,Q):
        r0T = rot(Q[0])*rot(Q[1])*rot(Q[2])
        p0T = rot(Q[0])*self.p01 + rot(Q[0])*rot(Q[1])*self.p12 + rot(Q[0])*rot(Q[1])*rot(Q[2])*self.p23
        return n.matrix([[r0T[0,0], r0T[0,1], p0T[0,0]],[r0T[1,0], r0T[1,1], p0T[1,0]],[0,0,1]])
    
    #The following function determines the position of each joint on the x,y plane.
    #This is useful for obstacle collision.
    def getJointPos(self,Q):
        O1 = rot(Q[0])*self.p01
        O2 = O1 + rot(Q[0])*rot(Q[1])*self.p12
        O3 = O2 + rot(Q[0])*rot(Q[1])*rot(Q[2])*self.p23
        return [O1,O2,O3]
        