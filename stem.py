#This class contains the Stem object. The stem object is an obstace 
#That the robot aims to avoid at all costs.
class Stem:
    #The cosntructor of the object. Stem takes in position coordinates and a radius
    def __init__(self,x,y,r):
        self.rad = r #declare radius
        self.x = x #declare x position
        self.y = y #declare y position
        
    #function returns the position of the stem    
    def getPoint(self):
        return (self.x,self.y)
    
    #function returns a boolean of whether a position (x,y) is in the bounds of the stem
    def isInside(self,x,y):
        #find the difference of the positions
        difx = x - self.x
        dify = y - self.y
        #Find the magnitude distance between the positions
        dist = (difx**2 + dify**2)**0.5
        #if the distance is less than the radius of the stem, then the object is 
        #colliding with the stem.
        if dist <= self.rad:
            return True
        else:
            return False 
        
    #Return the radius of the stem.
    def getRad(self):
        return self.rad
        