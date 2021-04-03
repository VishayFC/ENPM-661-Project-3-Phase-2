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

    def theta0(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        #print("parent ",node1)
        #print("child ",child)
        effort1 = effort1 + step_size
        return [child, effort1, direction_to_move]

    def theta30(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (math.pi/6)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        effort1 = effort1 + step_size
        return [child, effort1, direction_to_move]

    def theta60(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (math.pi/3)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        effort1 = effort1 + step_size
        return [child, effort1, direction_to_move]

    def theta_30(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 - (math.pi/6)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        effort1 = effort1 + step_size
        return [child, effort1, direction_to_move]

    def theta_60(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 - (math.pi/3)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        effort1 = effort1 + step_size
        return [child, effort1, direction_to_move]


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



#compares new children with goal state and adds them to the queue if the child is not the goal state
def compare_with_goal(ultimate_children, parent1):
    for child in ultimate_children:
        if child[0] == goal:
            print("\n Goal has been reached \n")
            return child[0], parent1, child[1]
        else:
            astar_cost = child[1] + euclidean_distance(child[0])
            astar_cost_list.append(astar_cost)
            astar_cost_list.sort(reverse = True)
            index_to_append_in_queue = astar_cost_list.index(astar_cost)
            duplicate_costqueue.insert(index_to_append_in_queue, child[1])
            queue1.add(node(child[0], parent1), index_to_append_in_queue)

    return None

###########
####check the location of the cost at visited list and append at that index in the queue
######cuurently there are errors in implementation


#main body of the code from this line and below
canvas_size = [302,402, 3]
canvas = np.zeros((canvas_size[0],canvas_size[1], canvas_size[2]))
visited_child_list = list()
visited_parent_list = list()
visited_child_cost = list()
astar_cost_list = list()
duplicate_costqueue = list()
duplicate_costcanvas = np.zeros((canvas_size[0],canvas_size[1], 1))
canvas = canvas.astype(np.uint8)
for_frames = list()


#marking obstacles
for i in range(canvas_size[0]):
    for j in range(canvas_size[1]):
        obstacles([i,j])




####################################
#################################
#the new node to be removed from the queue should be the one having least cost


#taking the start and goal node from the user and checking if in obstacle space
n = 1
while n > 0:
    start = list()
    goal = list()
    x1 = input("Enter the x co-ordinate of the start point: ")
    y1 = input("Enter the y co-ordinate of the start point: ")
    x2 = input("Enter the x co-ordinate of the goal point: ")
    y2 = input("Enter the y co-ordinate of the goal point: ")
    start.append(int(y1)+1)
    start.append(int(x1)+1)
    #start = np.array(start)
    #start.append(0)
    goal.append(int(y2)+1)
    goal.append(int(x2)+1)
    #goal = np.array(goal)
    #goal.append(1)
    lis = [start, goal]
    strt = list()
    count = 0
    for i in lis:
        strt.append(obstacles(i))
    if strt[0] == None or strt[1] == None:
        print("Error: One of the entered point is either in obstacle space or out of map boundary")
        continue
    else:
        n = 0

#canvas[301 - goal[0]][goal[1]][2] = 255


first_node = node(start, None)
queue1 = queue()
queue1.add(first_node,0)
duplicate_costqueue.append(0)


#calling the main functions of the Dijkstra
while True:
    new_parent = None
    while new_parent is None:
        new_node, cos = removing_from_queue()
        new_parent= check_if_visited(new_node, cos)
    children_list, parent = super_move_function(new_parent[0], new_parent[1])
    filtered_children, same_parent = check_if_in_obstacle_space(children_list, parent)
    child_parent = compare_with_goal(filtered_children, same_parent)
    if child_parent is not None:
        break

canvas[canvas_size[0] - goal[0], goal[1],:] = 128
cv2.imshow("window",canvas[1:301, 1:401])
#print(route)
print("final cost is: ", visited_child_cost[-1])
print("goal ",goal[0])



#print(duplicate_costcanvas)
#showing the obstacles in the canvas



'''
#visualization of the exploration of nodes
print("Video is saving....")



#visualizing the route to the goal node

'''
out.release()
print("\n Video file of visualization has been saved")
cv2.waitKey(0)
cv2.destroyAllWindows()