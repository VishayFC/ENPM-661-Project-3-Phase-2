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
    elif (st[1]+1 >= 200 and st[1]+1 <= 230 and st[0]+1 <= 280 and st[0]+1 >= 270):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (st[1]+1 >= 200 and st[1]+1 <= 230 and st[0]+1 <= 240 and st[0]+1 >= 230):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (st[0]+1) + (1.42814 * (st[1]+1)) >= 176.5511 and (st[0]+1) - (0.7 * (st[1]+1)) >= 74.39 and (st[0]+1) + (1.42814 * (st[1]+1)) <= 428.06815 and (st[0]+1) - (0.7 * (st[1]+1)) <= 98.80545:
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in rectangle")
        return None

    elif st[1] < 1 or st[1] >= canvas_size[1] - 1:
        canvas[st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    elif st[0] < 1  or st[0] >= canvas_size[0] - 1:
        canvas[st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None
    else :
        return st
    
    
#removes from the queue
def removing_from_queue():
    #print(queue1.size1())
    check = queue1.remove()
    cs = duplicate_costqueue.pop()
    #print("current ",check.current)
    #for_frames.append(visited_list)
    #print("queue size ",queue1.size())
    return check, cs

#calculate euclidean distance
def euclidean_distance(node):
    dist = ((goal[0] - node[0])**2 + (goal[1] - node[1])**2)**(1/2)
    return dist

def check_if_visited(check, cs):
    nod = check.current        #checking with the red value of canvas
    if canvas[(canvas_size[0] - 1) - nod[0],nod[1],2] == 255:
        if duplicate_costcanvas[(canvas_size[0] - 1) - nod[0],nod[1]] > cs:
            ind = visited_child_list.index(check.current)
            visited_parent_list[ind] = check.parent
            visited_child_cost[ind] = cs
            #print("replaced")

        ####################
        #############check if the cost previously computed is less or not if not then replace the parent
        return None
    canvas[(canvas_size[0] - 1) - nod[0], nod[1], 2] = 255    #marking visited by changing the color of red band
    duplicate_costcanvas[(canvas_size[0] - 1) - nod[0], nod[1],0] = cs
    #print(duplicate_costcanvas)
    visited_child_list.append(check.current)
    visited_parent_list.append(check.parent)
    visited_child_cost.append(cs)
    #cv2.imshow("window",canvas[1:301, 1:401])
    out.write(canvas[1:301, 1:401])
    #cv2.waitKey(1)
    return check, cs

#this function performs actions and gets children
def super_move_function(currentnode, cs):

    def theta30(node1, effort1, theta1):
        child = node1.copy()
        new_orient = theta1 + math.cos(math.pi/6)
        

        return

    def theta60(node1, effort1, theta1):

        return

    def theta_30(node1, effort1, theta1):

        return

    def theta_60(node1, effort1, theta1):

        return


    new_child = list()
    node = currentnode.current
    orient = currentnode.theta
    effort = cs
    new_child.append(moveleft(node, effort, orient))
    new_child.append(moveright(node, effort, orient))
    new_child.append(moveup(node, effort, orient))
    new_child.append(movedown(node, effort, orient))
    new_child.append(up_left(node, effort, orient))

    #new_child = np.array(new_child)

    return new_child, node

#checking if the node is in obstacle space and returns ones which are not in the obstacle space in a list and the parent node
def check_if_in_obstacle_space(children, parent1):
    valid_children = list()
    for i in children:
        if canvas[(canvas_size[0] - 1) - i[0][0], i[0][1], 0] == 255:
            continue
        valid_children.append(i)

    return valid_children, parent1
