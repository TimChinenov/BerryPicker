import node as nd
class Point:
    def __init__(node,coor,val):
        nd.node.__init__(coor)
        self.value = val
        
    def getVal(self):
        return self.value
    
    def setVal(self,val):
        self.value = val