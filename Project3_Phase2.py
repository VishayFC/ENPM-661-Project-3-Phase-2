import numpy as np
import cv2
import math

out = cv2.VideoWriter('Exploration.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 60, (400,300))

#defining the queue class to use as a data structure
class queue():
    def __init__(self):
        self.pending = list()

    def add(self, child, ind):
        self.pending.insert(ind, child)

    def remove(self):
        if self.pending:
            return self.pending.pop()
        return None

    def peek(self):
        if self.pending:
            return self.pending[-1]

    def size1(self):
        return len(self.pending)

    def isempty(self):
        if self.pending == []:
            return True
        return False
    
#created node class in order to save current node and it's parent
class node():
    def __init__(self, current, parent, theta):
        self.current = current
        self.parent = parent
        self.theta = theta
        #self.cost = cost 
        
#defining obstacles as well as the boundaries of the map
#st[0] = y coordinate in cartesian space     st[1] = x coordinate in cartesian space
def obstacles(st):
    if ((st[1]+1 - 90)**2 + (st[0]+1 - 70)**2) <= 1225:
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in circle")
        return None

    elif (((st[1]+1 - 246) / 60) ** 2) + (((st[0]+1 - 145) / 30) ** 2) <= 1:
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in ellipse")
        return None

    elif (st[1]+1 >= 200 and st[1]+1 <= 210 and st[0]+1 <= 280 and st[0]+1 >= 230):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None