import numpy as np
import random


'''Expandable, can calculate paths for any number of robots'''
n = 4 #Number of robots
Loop = range(n-1)
Conclude = range(n)
Distances1 = np.random.random((n,n)) #Placeholder for getPosition Distances
Paths = {}
Distances = {}
Distances[0] = Distances1
print(Distances1,'\n')
for x in Loop:
    print('Find path', x)
    i = x+1
    Paths[x] = min([min(path) for path in Distances[x]])
    s = np.where(Distances[x] == Paths[x])
    print('Remove row', s[0][0], '\nRemove column', s[1][0])
    Temp2 = np.delete(Distances[x], s[0][0], 0)
    Distances[i] = np.delete(Temp2, s[1][0], 1)#Change matrix from (n x n) to (n-1 x n-1)
    print(Distances[i])   
    print('Path ',i,' is', Paths[x],'\n')
Paths[n-1] = Distances[n-1][0][0]

#Show all paths
print('Path 4 is', Distances[n-1][0])
print('\n')
print('Our paths are:')
for x in Conclude:
    print('Path',x+1,'is', Paths[x])
   






