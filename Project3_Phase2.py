
import numpy as np
import cv2
import math

#out = cv2.VideoWriter('Exploration.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 60, (400,300))
out = cv2.VideoWriter('Output.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 60, (400,300))
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

    def show(self):
        for i in self.pending:
            print(i.current,"    ",((i.theta)*180)/math.pi)
        return

    def isempty(self):
        if self.pending == []:
            return True
        return False




#created node class in order to save current node and it's parent
class node():
    def __init__(self, current, parent, theta,par1_theta):
        self.current = current
        self.parent = parent
        self.theta = theta
        self.par1_theta = par1_theta



#defining obstacles as well as the boundaries of the map
#st[0] = y coordinate in cartesian space     st[1] = x coordinate in cartesian space


def obstacles(st):
    
    cl = radius + clearance
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    if ((st[1] - (90+padding)) ** 2) + ((st[0] - (70+padding)) ** 2) <= ((35+cl)**2):#((st[1] - 90+1)**2 + (st[0] - 70+1)**2) <= 1225:
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #print("coordinate is in circle")
        return None

    elif (((st[1] - (246+padding)) / (60+cl)) ** 2) + (((st[0] - (145+padding)) / (30+cl)) ** 2) <= 1:#(((st[1] - (246+1)) / 60) ** 2) + (((st[0] - (145)) / 30) ** 2) <= 1:
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #print("coordinate is in ellipse")
        return None

    #elif st[0] >=(230+1-cl) and st[1]>=(200+1-cl) and st[1]<=(230+1+cl) and st[0]<=(280+1+cl):#(st[1] >= 200+1 and st[1] <= 210+1 and st[0] <= 280+1 and st[0] >= 230+1):
        #canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        #return None

    elif (st[0] <= ((280+padding) + cl) and st[1]>=((200+padding)-cl) and st[0]>=((230+padding)-cl) and st[1]<=((230+padding)+cl)) and not (st[0]<=((270+padding)-cl) and st[1]>=((210+padding)+cl) and st[0]>=((240+padding)+cl) and st[1]<=((230+padding)+cl)):
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    #elif ((-0.7*(st[1]+padding))+(1*(st[0]+padding)))>=(73.4-a) and ((st[0]+10)+(1.42814*(st[1]+10)))>=(172.55-b) and ((-0.7*(st[1]+10))+(1*(st[0]+10)))<=(99.81+a) and ((st[0]+10)+(1.42814*(st[1]+10)))<=(429.07+b):
    elif (st[0] + (1.42814*st[1])) >= (188.6878 - b) and (st[0] - (0.7*st[1])) >= (75.8860 - a) and (st[0] + (1.42814*st[1])) <= (450.1952 + b) and (st[0] - (0.7*st[1])) <= (100.2985 + a):
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #print("coordinate is in rectangle")
        return None

    elif (st[1] >= ((canvas_size[1]-padding) - cl)) or (st[1] <= cl+padding):#st[1] < 1 or st[1] >= canvas_size[1] - 1:
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    elif (st[0] <= cl+padding) or (st[0] >= ((canvas_size[0]-padding) - cl)):#st[1] < 1 or st[1] >= canvas_size[1] - 1:
        canvas1[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        canvas[(canvas_size[0]-1)-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    else :
        return st



def goal_threshold():
    for i in range(canvas_size[0]):
        for j in range(canvas_size[1]):
            if ((j - goal[0][1])**2 + (i - goal[0][0])**2) <= (step_size)**2:
                canvas[(canvas_size[0]-1)-i,j,1] = 255

    return None

#removes from the queue
def removing_from_queue():
    #print(queue1.size1())
    #print("queue is: ")
    #queue1.show()
    check = queue1.remove()
    #print("node is ",check.current, "   ",(180*check.theta)/math.pi)
    #print(astar_cost_list)

    cs = duplicate_costqueue.pop()
    #print("astar cost is ",astar_cost_list)
    #print("theta ",check.theta)
    acs = astar_cost_list.pop()
    #print("cost to come: ",cs)
    #print("astar cost ",acs)
    #print("current ",check.current)
    #print("current ",check.current)
    #for_frames.append(visited_list)
    #print("queue size ",queue1.size())
    return check, cs

#calculate euclidean distance
def euclidean_distance(node):
    dist = ((goal[0][0] - node[0])**2 + (goal[0][1] - node[1])**2)**(1/2)
    return dist



#checking if the node is in the queue or has been visited previously and then appending the parent to the visited_list
def check_if_visited(check, cs):
    nod = check.current        #checking with the red value of canvas
    if canvas1[(canvas_size[0]-1) - nod[0],nod[1],2] == 255:
        orient_info = canvas1[(canvas_size[0]-1) - nod[0],nod[1],:]
        orient_ind = np.where(orient_info == 2)
        #print("same xy")
        for i in orient_ind:
            for j in i:
                if angle_list[j] == round((180*check.theta)/math.pi):
                    if duplicate_costcanvas[(canvas_size[0]-1) - nod[0],nod[1]] > cs:
                    #if duplicate_costcanvas[(canvas_size[0]-1) - nod[0],nod[1]] + euclidean_distance(check.current) > duplicate_costcanvas[(canvas_size[0]-1) - check.parent[0],check.parent[1]] + 1 + euclidean_distance(check.parent):
                        ind = visited_child_list.index([check.current,round(check.theta,8)])
                        visited_parent_list[ind] = [check.parent,round(check.par1_theta,8)]
                        visited_child_cost[ind] = cs
                        #print("replaced")

                    ####################
                    #############check if the cost previously computed is less or not if not then replace the parent
                    return None
    canvas1[(canvas_size[0]-1) - nod[0], nod[1], 2] = 255    #marking visited by changing the color of red band
    #print("check ",round(check.theta))
    ind = angle_list.index(round((180*check.theta)/math.pi))
    #print(ind)
    canvas1[(canvas_size[0]-1) - nod[0], nod[1], ind] = 2
    duplicate_costcanvas[(canvas_size[0]-1) - nod[0], nod[1],0] = cs
    if check.parent is not None:
        pary = check.parent[0]
        cury = check.current[0]
        cv2.line(canvas, (check.parent[1],(canvas_size[0]-1)-pary), (check.current[1],(canvas_size[0]-1)-cury), (0,0,255), 1)
    #print(duplicate_costcanvas)
    if check.parent == None:
        visited_child_list.append([check.current,round(check.theta,8)])
        visited_parent_list.append([check.parent,check.par1_theta])
        visited_child_cost.append(cs)
    else:
        visited_child_list.append([check.current,round(check.theta,8)])
        visited_parent_list.append([check.parent,round(check.par1_theta,8)])
        visited_child_cost.append(cs)

    out.write(canvas[padding:(300+padding),padding:(400+padding)])
    #cv2.waitKey(1)
    return check, cs






#this function performs actions and gets children
def super_move_function(currentnode, cs):

    def theta0(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))

        effort1 = effort1 + 1
        if direction_to_move >= (6.2831):
            direction_to_move = direction_to_move - (2*math.pi)
        return [child, effort1, direction_to_move]

    def theta30(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (math.pi/6)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        #print("child 30 ",child)
        effort1 = effort1 + 1
        if direction_to_move >= (6.2831):
            direction_to_move = direction_to_move - (2*math.pi)
        return [child, effort1, direction_to_move]

    def theta60(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (math.pi/3)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        #print("child 60 ",child)
        effort1 = effort1 + 1
        if direction_to_move >= (6.2831):
            direction_to_move = direction_to_move - (2*math.pi)
        return [child, effort1, direction_to_move]

    def theta_30(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (330*math.pi/180)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        #print("child -30 ",child)
        effort1 = effort1 + 1
        if direction_to_move >= (6.2831):
            direction_to_move = direction_to_move - (2*math.pi)
        return [child, effort1, direction_to_move]

    def theta_60(node1, effort1, theta1):
        child = node1.copy()
        direction_to_move = theta1 + (300*math.pi/180)
        child[0] = round(child[0] + (step_size* math.sin(direction_to_move)))
        child[1] = round(child[1] + (step_size* math.cos(direction_to_move)))
        #print("child -60 ",child)
        effort1 = effort1 + 1 #step_size
        if direction_to_move >= (6.2831):
            direction_to_move = direction_to_move - (2*math.pi)
        return [child, effort1, direction_to_move]


    new_child = list()
    node = currentnode.current
    orient = currentnode.theta
    effort = cs
    new_child.append(theta60(node, effort, orient))
    new_child.append(theta30(node, effort, orient))
    new_child.append(theta0(node, effort, orient))
    new_child.append(theta_30(node, effort, orient))
    new_child.append(theta_60(node, effort, orient))

    #new_child = np.array(new_child)

    return new_child, node, orient

#checking if the node is in obstacle space and returns ones which are not in the obstacle space in a list and the parent node
def check_if_in_obstacle_space(children, parent1):
    valid_children = list()
    for i in children:
        #print(i)
        #print((canvas_size[0] - 1) - i[0][0])
        if canvas1[(canvas_size[0]-1) - i[0][0], i[0][1], 0] == 255:
            #print(i)
            continue
        valid_children.append(i)

    return valid_children, parent1




#compares new children with goal state and adds them to the queue if the child is not the goal state
def compare_with_goal(ultimate_children, parent1,par_theta):
    for child in ultimate_children:
        #print("new child ", child[0][0],child[0][1])
        if canvas[(canvas_size[0]-1) - child[0][0], child[0][1], 1] == 255:
            print(child[0])
            print("\n Goal has been reached \n")
            return [child[0], round(child[2],8)], [parent1,round(par_theta,8)], child[1]
        else:
            astar_cost = child[1] + euclidean_distance(child[0])
            astar_cost_list.append(astar_cost)
            astar_cost_list.sort(reverse = True)
            index_to_append_in_queue = astar_cost_list.index(astar_cost)
            duplicate_costqueue.insert(index_to_append_in_queue, child[1])
            queue1.add(node(child[0], parent1, child[2],par_theta), index_to_append_in_queue)

    return None



#main body of the code from this line and below
padding = 10
threshold = 0.5
canvas_size = [300+(2*padding),400+(2*padding), 3]
canvas = np.zeros((canvas_size[0],canvas_size[1], canvas_size[2]))
canvas1 = np.zeros((canvas_size[0],canvas_size[1], 3+12))
visited_canvas = np.zeros((int(canvas_size[0]/threshold),int(canvas_size[1]/threshold),12))
visited_child_list = list()
visited_parent_list = list()
visited_child_cost = list()
astar_cost_list = list()
duplicate_costqueue = list()
duplicate_costcanvas = np.zeros((canvas_size[0],canvas_size[1], 1))
canvas = canvas.astype(np.uint8)
for_frames = list()
angle_list = [10,10,10, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]

radius = int(input("Enter the radius: "))
clearance = int(input("Enter the clearance: "))
#marking obstacles
for i in range(canvas_size[0]):
    for j in range(canvas_size[1]):
        obstacles([i,j])




#taking the start and goal node from the user and checking if in obstacle space
n = 1
while n > 0:
    start = list()
    goal = list()
    x1 = input("Enter the x co-ordinate of the start point: ")
    y1 = input("Enter the y co-ordinate of the start point: ")
    st_theta = input("Enter the orientation of start point: ")
    x2 = input("Enter the x co-ordinate of the goal point: ")
    y2 = input("Enter the y co-ordinate of the goal point: ")
    gl_theta = input("Enter the orientation of goal point: ")
    step_size = input("Enter the step size: ")
    start.append([int(y1)+padding,int(x1)+padding])
    start.append((int(st_theta)*math.pi)/180)

    #start = np.array(start)
    #start.append(0)
    goal.append([int(y2)+padding,int(x2)+padding])
    goal.append((int(gl_theta)*math.pi)/180)
    
    step_size = float(step_size)
    lis = [start, goal]
    strt = list()
    count = 0
    for i in lis:
        strt.append(obstacles(i[0]))
    if strt[0] == None or strt[1] == None:
        print("Error: One of the entered point is either in obstacle space or out of map boundary")
        continue
    else:
        n = 0



canvas[(canvas_size[0]-1)- goal[0][0], goal[0][1],1] = 255
first_node = node(start[0], None, start[1],None)
queue1 = queue()
queue1.add(first_node,0)
duplicate_costqueue.append(0)
astar_cost_list.append(euclidean_distance(start[0]))
goal_threshold()                        #defines the area according to the threshold as goal region


#calling the main functions of the Dijkstra
while True:
    new_parent = None
    while new_parent is None:
        new_node, cos = removing_from_queue()
        new_parent= check_if_visited(new_node, cos)
    children_list, parent, parent_theta = super_move_function(new_parent[0], new_parent[1])
    filtered_children, same_parent = check_if_in_obstacle_space(children_list, parent)
    child_parent = compare_with_goal(filtered_children, same_parent, parent_theta)
    if child_parent is not None:
        break

visited_child_list.append(child_parent[0])
visited_parent_list.append(child_parent[1])
visited_child_cost.append(child_parent[2])
parent_info = child_parent[1]


#searching for the route to the goal node or backtracking
route = list()


#print(visited_child_list)
while parent_info is not None:
    for i in visited_child_list:
        if parent_info == i:
            
            parent_info = visited_parent_list[visited_child_list.index(i)]
            if parent_info[0] is None:
                break
            else:
                
                cv2.line(canvas, (i[0][1],(canvas_size[0]-1)-i[0][0]), (parent_info[0][1],(canvas_size[0]-1)-parent_info[0][0]), (128,128,128), 2)
            out.write(canvas[padding:(300+padding),padding:(400+padding)])

            break
    if parent_info[0] is None:
        break


canvas[(canvas_size[0]-1)- goal[0][0], goal[0][1],1] = 255


cv2.imshow("window",canvas[padding:(300+padding),padding:(400+padding)])





out.release()
print("\n Video file of visualization has been saved")
cv2.waitKey(0)
cv2.destroyAllWindows()
