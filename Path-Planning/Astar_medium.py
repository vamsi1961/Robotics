
import matplotlib.pyplot as plt
import math
import time

show_animation = True
class AStarPlanner:

    def __init__(self, ox, oy, resolution, robot_radius):

        self.x_min = 0
        self.y_min = 0
        self.x_max = 0
        self.y_max = 0
        self.x_width = 0
        self.y_width = 0
        self.obstacle_map = 0

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)


    def planning(self,sx,sy,gx,gy):
        
        start_node = self.Node(self.calc_xy_index(sx, self.x_min),self.calc_xy_index(sy, self.y_min), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.x_min),self.calc_xy_index(gy, self.y_min), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        print(str(start_node))

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            curr_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,open_set[o]))
            current = open_set[curr_id]
            print(current.cost)

             #show_graph

            # if show_animation:
            #     plt.plot(self.calc_position(current.x,self.x_min),self.calc_position(current.y,self.y_min),"xc")

            #     plt.gcf().canvas.mpl_connect('key_release_event', lambda event:[exit(0) if event.key == 'escape' else None])

            #     if len(closed_set.keys()) % 10 ==  0:
            #         plt.pause(0.01)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find Goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[curr_id]

            closed_set[curr_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], curr_id)

                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node

                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node 
        
        rx,ry = self.calc_final_path(goal_node,closed_set)
        # returns distance 

        return rx,ry 

    def calc_position(self,index,minp):
        pos = index*self.resolution + minp
        return pos


    def calc_final_path(self,goal_node,closed_set):

        rx,ry = [self.calc_position(goal_node.x ,self.x_min)], [self.calc_position(goal_node.y,self.y_min)]
        parent_index = goal_node.parent_index
        print(goal_node.cost)
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x,self.x_min))
            ry.append(self.calc_position(n.y,self.y_min))
            parent_index = n.parent_index

        return rx,ry

    def calc_index(self, node):
        return (node.y - self.y_min) * self.x_width + (node.x - self.x_min)
    
    def calc_xy_index(self,position,minp):
        return round((position - minp)/self.resolution)

    @staticmethod
    def calc_heuristic(n1,n2):

        #d = abs(n1.x-n2.x) + abs(n1.y-n2.y)
        if n2.x >= 20  and n2.x <= 80 and n2.y <= 50:
            d = max(abs(n1.x-n2.x) ,abs(n1.y-n2.y))*2
        else:
            d = max(abs(n1.x-n2.x) ,abs(n1.y-n2.y))
        # d = max(abs(n1.x - n2.x), abs(n1.y - n2.y)) * math.sqrt(2)
        #d = w*math.hypot(n1.x - n2.x , n1.y - n2.y)
        return d

    def verify_node(self,node):

        px = self.calc_position(node.x,self.x_min)
        py = self.calc_position(node.y,self.y_min)

        if px < self.x_min:
            return False

        if py < self.y_min:
            return False

        if px >= self.x_max:
            return False

        if py >= self.y_max:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True


    def calc_obstacle_map(self,ox,oy):
        
        self.x_min = round(min(ox))
        self.x_max = round(max(ox))
        self.y_min = round(min(oy))
        self.y_max = round(max(oy))
        print("x_min:", self.x_min)
        print("y_min:", self.y_min)
        print("x_max:", self.x_max)
        print("y_max:", self.y_max)

        self.x_width = round((self.x_max - self.x_min) / self.resolution)
        self.y_width = round((self.y_max - self.y_min) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]

        for ix in range(self.x_width):
            x = self.calc_position(ix,self.x_min)
            for iy in range(self.y_width):
                y = self.calc_position(iy,self.y_min)
                for iox,ioy in zip(ox,oy):
                    d = math.hypot(iox-x,ioy -y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        #dx,dy,cost
        motion = [
            [1,0,1],
            [0,1,1],
            [-1,0,1],
            [0,-1,1],
            [-1,-1,math.sqrt(2)],
            [-1,1,math.sqrt(2)],
            [1,-1,math.sqrt(2)],
            [1,1,math.sqrt(2)]
        ]

        return motion

    
def main():
    print( __file__ + "start!!" )

    # start and goal position
    sx = 10.0   #[m]
    sy = 45.0   #[m]
    gx = 90.0   #[m]
    gy = 45.0   #[m]
    grid_size = 1.0
    robot_radius = 1.0
    # set obstacle positions
    ox,oy =[],[]
    hx,hy = [],[]
    
    for i in range(20,81,10):
        for j in range(0,51,10):
            print(i,j)
            hx.append(i)
            hy.append(j)


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
        plt.plot(hx,hy, "-c")
        plt.plot(gx,gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    start = time.time()
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    end = time.time()
    print(end - start)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
