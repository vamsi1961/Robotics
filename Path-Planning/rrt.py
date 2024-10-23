import math
import numpy as np
import utils
import plotting
import env
import time

class Node:
    def __init__(self,n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRT:

    def __init__(self,start,goal,step_len,goal_sample_rate, iter_max):
        self.s_start = Node(start)
        self.s_goal = Node(goal)
        self.s_step = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(start,goal)
        self.utils = utils.Utils()

# environment variables

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary


    def planning(self):
        for i in range(self.iter_max):
            newNode = self.generate_random_node(self.goal_sample_rate)
            nodeNear = self.nearest_neghbor(self.vertex,newNode)
            nodeNew = self.new_state(nodeNear,newNode)

            if nodeNew and not self.utils.is_collision(nodeNear,nodeNew):
                self.vertex.append(nodeNew)
                dist , _ = self.get_distance_and_angle(nodeNew, self.s_goal)

                if dist <= self.s_step and not self.utils.is_collision(nodeNew,self.s_goal):
                    self.new_state(nodeNew,self.s_goal)
                    return self.extract_path(newNode)
                
        return None

    def generate_random_node(self,goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neghbor(node_list,n):
        return node_list[int(np.argmin([math.hypot(nd.x-n.x, nd.y-n.y) for nd in node_list]))]

    def new_state(self,node_start,node_end):
        dist,angle = self.get_distance_and_angle(node_start, node_end)
        dist = min(dist, self.s_step)
        node_new = Node((node_start.x + dist*math.cos(angle) , node_start.y + dist*math.sin(angle)))

        node_new.parent = node_start
        return node_new

    def extract_path(self,node_end):
        path = [(self.s_goal.x,self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start,node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y

        # Distance calculation using NumPy
        dist = np.hypot(dx, dy)

        # Angle calculation using NumPy
        angle = np.arctan2(dy, dx)

        return dist, angle






def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt = RRT(x_start, x_goal, 0.5, 0.05, 10000)
    path = rrt.planning()

    if path:
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()



