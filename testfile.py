import bot 
import numpy as np

robot1 = bot.Bot(2.0,1.5,1.0) #robot object
M = np.matrix([[1,0,1],[0,1,3.5],[0, 0, 1]])
Q = robot1.invKin(M)
print Q
for qs in Q:
    print robot1.fwdKin(qs)
#import numpy as np
#import matplotlib.pyplot as plt

#plt.axis([0, 10, 0, 1])
#plt.ion()

#for i in range(10):
    #y = np.random.random()
    #plt.scatter(i, y)
    #plt.pause(0.05)

#while True:
    #plt.pause(0.05)
    