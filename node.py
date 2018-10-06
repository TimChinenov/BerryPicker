#This is the node class. The node class is an object tat is used 
#to generate paths. A series of nodes are attached together to 
#make a path.
class node:
    #initialize the node
    def __init__(self,coor):
        #give the node coordinates
        self.point = coor
        #set its parent to none
        self.parent = None
        #set children to empty
        self.children = []
    #return the position of the node    
    def getPoint(self):
        return self.point
    #return the children of the node
    def getChildren(self):
        return self.children
    #return the parent of the node
    #nodes only have one parent.
    def getParents(self):
        return self.parent
    
    #add a child node to list of children
    def addChild(self,child):
        self.children.append(child)
    #add a parent node    
    def addParent(self,par):
        self.parent = par