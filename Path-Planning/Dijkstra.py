
import matplotlib.pyplot as plt
import math
import time
show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None
        # Initially setup requirements according to grid which we plot in matplolib 

        # as we convert meters to pixels we use gridsize as resolution
        self.resolution = resolution
        #  as our robot moves it has to understand wether it hits obstacle so we have to check wethere obstacle or goal is in radius
        self.robot_radius = robot_radius
        # we create obstacles in the map 
        self.calc_obstacle_map(ox, oy)
        # we choose motion where to move 
        self.motion = self.get_motion_model()


    class Node:
        def __init__(self, x, y, cost, parent_index):
            # define each point as node
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost # based on cost we find shortest path 
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)


    def planning(self,sx,sy,gx,gy):
        
        # we create obstacles in the map 
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # open set to log all nodes which are yet to explore.
        # closed  set to log all nodes which are explored.
        open_set, closed_set = dict(), dict()
        # calc_index function distinguishes each node it returns different value for each location of node 
        #  it can be any function but make sure it distinguishes each node  
        open_set[self.calc_index(start_node)] = start_node
        # start_node is added to the open_set
        print(str(start_node))

        while True:
            # we have to check which node is near in terms of cost
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            # we compare costs and then find minimum and then delete which is closest.
            # for each node cost will get updated either it is in open_set or not
            # c_id is current node which we will explore next 
            current = open_set[c_id]

            #  show_graph

            # if show_animation:
            #     plt.plot(self.calc_position(current.x,self.min_x),self.calc_position(current.y,self.min_y),"xc")

            #     plt.gcf().canvas.mpl_connect('key_release_event', lambda event:[exit(0) if event.key == 'escape' else None])

            #     if len(closed_set.keys()) % 10 ==  0:
            #         plt.pause(0.001)

            # if the current node is goal node then stop you found the goal
            # As cost is accumulated final cost = cost  

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find Goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # when we explored the current node and we are moving further we delete that node from open_set and we add it in closed_set node   
            del open_set[c_id]

            # when we found the closest node we add it in closed_set 
            closed_set[c_id] = current


            # we have to explore all moves  
            for move_x,move_y,move_cost in self.motion:
                # there are 8 moves in which x,y,cost are available
                # we will find current node after moving and new x,y,cost and parent node are passed
                node = self.Node(current.x + move_x , current.y + move_y, current.cost + move_cost ,c_id )
                
                # find index of the node
                n_id = self.calc_index(node)

                # if it is already in closed_set then continue to start 
                # because it will be in loop if you again assume it as closest point
                if n_id in closed_set:
                    continue
                
                # verify if it is a node or not
                if not self.verify_node(node):
                    continue
                
                # if new node is not in open_set then add it  
                if n_id not in open_set:
                    open_set[n_id] = node
                # if it is in open_set then compare and replace if the node is of less cost, 
                # if it is more cost then we can use previous xost and parent nodes as x,y is same 
                #  i.e we are replacing the cost and parent node 
                else:
                    if open_set[n_id].cost >= node.cost:
                        open_set[n_id] = node 

        print(len(closed_set))

        # after exploring we calculate final path
        rx,ry = self.calc_final_path(goal_node,closed_set)

        # returns distance 

        return rx,ry 

    # to convert index to position  
    def calc_position(self,index,minp):
        pos = index*self.resolution + minp
        return pos

    
    # after exploring we got goal_node and closed_set points now find final path  
    def calc_final_path(self,goal_node,closed_set):
        
        # we will travel back wards first we will find position of goal
        rx,ry = [self.calc_position(goal_node.x ,self.min_x)], [self.calc_position(goal_node.y,self.min_y)]
        # find it's parent node 
        parent_index = goal_node.parent_index
        print(goal_node.cost)
        while parent_index != -1:

            # go backwards and append rx,ry
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x,self.min_x))
            ry.append(self.calc_position(n.y,self.min_y))
            parent_index = n.parent_index

        return rx,ry

    # this gives a unique value for wach node and stores node in corresponding node
    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    
    # convert distance to grid
    def calc_xy_index(self,position,minp):
        return round ((position - minp)/self.resolution)

    
    # verify node wether it is in given conditions or not
    def verify_node(self,node):

        px = self.calc_position(node.x,self.min_x)
        py = self.calc_position(node.y,self.min_y)

        if px < self.min_x:
            return False

        if py < self.min_y:
            return False

        if px >= self.max_x:
            return False

        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    

    def calc_obstacle_map(self,ox,oy):
        
        self.min_x = round(min(ox))
        self.max_x = round(max(ox))
        self.min_y = round(min(oy))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        # find x,y widths 

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # initially all points in grid are False 
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        
        # we finally 
        for ix in range(self.x_width):
            x = self.calc_position(ix,self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy,self.min_y)
                #The zip function is used to iterate over the elements of ox and oy at the same time, 
                #so that the x and y coordinates of each obstacle can be used in the collision check.
                for iox,ioy in zip(ox,oy):
                    d = math.hypot(iox-x,ioy -y)
                    # checks whether robot collides or not     
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        #dx,dy,cost
        motion = [
            [1,0,1],
            
            [-1,0,1],
            [0,-1,1],
            [-1,-1,math.sqrt(2)],
            [-1,1,math.sqrt(2)],
            [1,-1,math.sqrt(2)],
            [0,1,1],
            [1,1,math.sqrt(2)]]

        return motion


def main():
    print( __file__ + "start!!" )

    # start and goal position
    sx = 10.0   #[m]
    sy = 50.0   #[m]
    gx = 90.0   #[m]
    gy = 50.0   #[m]
    grid_size = 1.0
    robot_radius = 1.0
    # set obstacle positions
    ox,oy =[],[]

    for i in range(0,101): 
        ox.append(i)
        oy.append(0.0)

    for i in range(0,101): 
        ox.append(i)
        oy.append(100.0)

    for i in range(0,101):
        ox.append(100.0)
        oy.append(i)
    
    for i in range(0,101):
        ox.append(0.0)
        oy.append(i)

    for i in range(30,70):
        ox.append(i)
        oy.append(50)

 
    if show_animation:
        plt.plot(ox,oy, ".k")
        plt.plot(sx,sy, "og")
        plt.plot(gx,gy, "xb")
        plt.grid(True)
        plt.axis("equal")


    dijkstra = Dijkstra(ox,oy,grid_size,robot_radius)

    rx , ry = dijkstra.planning(sx,sy,gx,gy)

    if show_animation:
        plt.plot(rx,ry, "-r")
        plt.pause(0.01)
        plt.show()

if __name__ == '__main__':
    main()

















