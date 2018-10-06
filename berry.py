#Berry is a node object. The berry only contains a x,y position
class Berry:
    def __init__(self,x,y):
        #assign position to the berry
        self.x = x
        self.y = y
    
    #Return the position of the berry.    
    def getPoint(self):
        return (self.x,self.y)
    	