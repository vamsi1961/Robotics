import numpy as np
import matplotlib.pyplot as plt
import math

show_animation = True


class Node:
    """A class representing a node with properties of g, h, coor, and a parent node"""

    def __init__(self, g_cost=0, h_cost=0, coor=None, parent_node=None):
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent_node = parent_node
        self.coor = coor

    def reset_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost


def heuristic_cost(coor, goal_coor):
    dx = abs(coor[0] - goal_coor[0])
    dy = abs(coor[1] - goal_coor[1])
    h_cost = dx + dy
    return h_cost


def movement_cost(current_node, new_coor):
    dx = abs(current_node.coor[0] - new_coor[0])
    dy = abs(current_node.coor[1] - new_coor[1])
    movement_cost = math.hypot(dx, dy)
    g_cost = current_node.g_cost + movement_cost
    return g_cost


def generate_boundary_and_obstacles(start_coor, goal_coor, top_right_vertex, bottom_left_vertex, obstacle_count):
    """
    Generate boundary and obstacles in the map.

    :param start_coor: Coordinates of the starting point
    :param goal_coor: Coordinates of the goal
    :param top_right_vertex: Coordinates of the top right vertex of the boundary
    :param bottom_left_vertex: Coordinates of the bottom left vertex of the boundary
    :param obstacle_count: Number of obstacles to be generated in the map
    :return: Array representing the boundary and a list of obstacle coordinates
    """

    # Define coordinates for the four sides of the boundary
    top_y = list(range(bottom_left_vertex[1], top_right_vertex[1]))
    top_x = [bottom_left_vertex[0]] * len(top_y)

    bottom_y = top_y
    bottom_x = [top_right_vertex[0]] * len(bottom_y)

    left_x = list(range(bottom_left_vertex[0] + 1, top_right_vertex[0]))
    left_y = [bottom_left_vertex[1]] * len(left_x)

    right_x = [bottom_left_vertex[0]] + left_x + [top_right_vertex[0]]
    right_y = [top_right_vertex[1]] * len(right_x)

    # Generate random obstacles
    obstacle_x = np.random.randint(bottom_left_vertex[0] + 1, top_right_vertex[0], obstacle_count).tolist()
    obstacle_y = np.random.randint(bottom_left_vertex[1] + 1, top_right_vertex[1], obstacle_count).tolist()

    # Combine the coordinates of the boundary and obstacles
    x_coordinates = top_x + left_x + bottom_x + right_x
    y_coordinates = top_y + left_y + bottom_y + right_y
    obstacles = np.vstack((obstacle_x, obstacle_y)).T.tolist()

    # Remove the start and goal coordinates from the obstacle list
    obstacles = [obstacle for obstacle in obstacles if obstacle != start_coor and obstacle != goal_coor]
    obstacles_array = np.array(obstacles)
    boundary = np.vstack((x_coordinates, y_coordinates)).T
    boundary_and_obstacles = np.vstack((boundary, obstacles_array))

    return boundary_and_obstacles, obstacles


def find_valid_neighbors(current_node, obstacle_coordinates, closed_nodes):
    # Generate valid neighbors based on certain conditions
    obstacle_list = obstacle_coordinates.tolist()
    neighbors = []

    for x in range(current_node.coor[0] - 1, current_node.coor[0] + 2):
        for y in range(current_node.coor[1] - 1, current_node.coor[1] + 2):
            if [x, y] not in obstacle_list:
                # Find all possible neighbor nodes
                neighbors.append([x, y])

    # Remove nodes that violate the motion rules
    # 1. Remove the node with the same coordinates as the current node
    neighbors.remove(current_node.coor)

    # 2. Remove neighbor nodes that cross through two diagonally positioned obstacles
    # since there is not enough space for the robot to go through two diagonal obstacles
    top_neighbor = [current_node.coor[0], current_node.coor[1] + 1]
    bottom_neighbor = [current_node.coor[0], current_node.coor[1] - 1]
    left_neighbor = [current_node.coor[0] - 1, current_node.coor[1]]
    right_neighbor = [current_node.coor[0] + 1, current_node.coor[1]]

    # Neighbors in the four corners
    top_left_neighbor = [current_node.coor[0] - 1, current_node.coor[1] + 1]
    top_right_neighbor = [current_node.coor[0] + 1, current_node.coor[1] + 1]
    bottom_left_neighbor = [current_node.coor[0] - 1, current_node.coor[1] - 1]
    bottom_right_neighbor = [current_node.coor[0] + 1, current_node.coor[1] - 1]

    # Remove unnecessary neighbors
    if top_neighbor in obstacle_list and left_neighbor in obstacle_list and top_left_neighbor in neighbors:
        neighbors.remove(top_left_neighbor)
    if top_neighbor in obstacle_list and right_neighbor in obstacle_list and top_right_neighbor in neighbors:
        neighbors.remove(top_right_neighbor)
    if bottom_neighbor in obstacle_list and left_neighbor in obstacle_list and bottom_left_neighbor in neighbors:
        neighbors.remove(bottom_left_neighbor)
    if bottom_neighbor in obstacle_list and right_neighbor in obstacle_list and bottom_right_neighbor in neighbors:
        neighbors.remove(bottom_right_neighbor)

    # Remove neighbors that are already in the closed nodes
    neighbors = [neighbor for neighbor in neighbors if neighbor not in closed_nodes]

    return neighbors


def find_node_index_by_coordinates(coor, node_list):
    # Find the index of a node in the node list based on its coordinates
    index = 0
    for node in node_list:
        if node.coor == coor:
            target_node = node
            index = node_list.index(target_node)
            break
    return index


def find_path(open_nodes, closed_nodes, goal_coor, obstacle_coordinates):
    # Search for the path and update the open and closed nodes
    obstacle_list = obstacle_coordinates.tolist()
    for i in range(len(open_nodes)):
        current_node = open_nodes[0]
        open_coordinates_list = [node.coor for node in open_nodes]
        closed_coordinates_list = [node.coor for node in closed_nodes]

        valid_neighbors = find_valid_neighbors(current_node, obstacle_coordinates, closed_coordinates_list)

        for neighbor_coor in valid_neighbors:
            if neighbor_coor in closed_nodes:
                continue
            elif neighbor_coor in open_coordinates_list:
                # If the neighbor is already in the open list, update its g value
                neighbor_index = open_coordinates_list.index(neighbor_coor)
                new_g_cost = movement_cost(current_node, neighbor_coor)
                if new_g_cost <= open_nodes[neighbor_index].g_cost:
                    open_nodes[neighbor_index].g_cost = new_g_cost
                    open_nodes[neighbor_index].reset_f_cost()
                    open_nodes[neighbor_index].parent_node = current_node
            else:
                # Create a new node for the neighbor
                neighbor_node = Node(coor=neighbor_coor, parent_node=current_node,
                                     g_cost=movement_cost(current_node, neighbor_coor),
                                     h_cost=heuristic_cost(neighbor_coor, goal_coor))
                open_nodes.append(neighbor_node)

        open_nodes.remove(current_node)
        closed_nodes.append(current_node)
        open_nodes.sort(key=lambda x: x.f_cost)

    return open_nodes, closed_nodes


def nodes_to_coordinates(node_list):
    # Convert a list of nodes to a list of coordinates
    coordinates_list = [node.coor for node in node_list]
    return coordinates_list


def find_intersecting_nodes(closed_list1, closed_list2):
    # Find nodes that intersect between two closed lists
    coordinates_list1 = nodes_to_coordinates(closed_list1)
    coordinates_list2 = nodes_to_coordinates(closed_list2)
    intersecting_nodes = [node for node in coordinates_list1 if node in coordinates_list2]
    return intersecting_nodes


def find_obstacles_around_node(coor, obstacle_coordinates):
    # Find obstacles around a node to help draw the borderline
    obstacles_around_node = []
    for x in range(coor[0] - 1, coor[0] + 2):
        for y in range(coor[1] - 1, coor[1] + 2):
            if [x, y] in obstacle_coordinates:
                obstacles_around_node.append([x, y])
    return obstacles_around_node


def get_border_line(closed_node_list, obstacle_coordinates):
    # Find the border line that confines the goal or robot if there is no path
    border_line = []
    coordinate_list = nodes_to_coordinates(closed_node_list)
    for coor in coordinate_list:
        obstacles_around_node = find_obstacles_around_node(coor, obstacle_coordinates)
        border_line += obstacles_around_node
    border_line_array = np.array(border_line)
    return border_line_array


def get_path_from_nodes(open_list, closed_list, coor):
    # Get the path from start to end
    path_from_start = []
    path_from_end = []
    index = find_node_index_by_coordinates(coor, open_list)
    node = open_list[index]
    while node != open_list[0]:
        path_from_start.append(node.coor)
        node = node.parent_node
    path_from_start.append(open_list[0].coor)
    index = find_node_index_by_coordinates(coor, closed_list)
    node = closed_list[index]
    while node != closed_list[0]:
        path_from_end.append(node.coor)
        node = node.parent_node
    path_from_end.append(closed_list[0].coor)
    path_from_start.reverse()
    path = path_from_start + path_from_end
    path = np.array(path)
    return path


def generate_random_coordinates(bottom_left_vertex, top_right_vertex):
    # Generate random coordinates inside the maze
    coordinates = [np.random.randint(bottom_left_vertex[0] + 1, top_right_vertex[0]),
                   np.random.randint(bottom_left_vertex[1] + 1, top_right_vertex[1])]
    return coordinates


def plot_map(closed_origin, closed_goal, start_coor, goal_coor, boundary_coordinates):
    # Plot the map
    if not closed_goal.tolist():  # Ensure the closed_goal is not empty
        closed_goal = np.array([goal_coor])

    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(closed_origin[:, 0], closed_origin[:, 1], 'oy')
    plt.plot(closed_goal[:, 0], closed_goal[:, 1], 'og')
    plt.plot(boundary_coordinates[:, 0], boundary_coordinates[:, 1], 'sk')
    plt.plot(goal_coor[0], goal_coor[1], '*b', label='Goal')
    plt.plot(start_coor[0], start_coor[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)


def draw_control(origin_closed, goal_closed, status, start_coor, goal_coor, boundary_coordinates, obstacle_coordinates):
    """
    Control the plotting process, evaluate if the searching is finished.
    status: 0 (not blocked), 1 (start point blocked), 2 (end point blocked)
    """

    stop_searching = 0  # Stop sign for the searching
    origin_closed_coordinates = nodes_to_coordinates(origin_closed)
    origin_array = np.array(origin_closed_coordinates)
    goal_closed_coordinates = nodes_to_coordinates(goal_closed)
    goal_array = np.array(goal_closed_coordinates)
    path = None

    if show_animation:  # Draw the searching process
        plot_map(origin_array, goal_array, start_coor, goal_coor, boundary_coordinates)

    if status == 0:  # A path has been found
        intersecting_nodes = find_intersecting_nodes(origin_closed, goal_closed)
        if intersecting_nodes:
            path = get_path_from_nodes(origin_closed, goal_closed, intersecting_nodes[0])
            stop_searching = 1
            print('Path found!')
            if show_animation:  # Draw the path
                plt.plot(path[:, 0], path[:, 1], '-r')
                plt.title('Breadth First Search', size=20, loc='center')
                plt.pause(0.01)
                plt.show()

    elif status == 1:  # Start point is blocked
        stop_searching = 1
        print('There is no path to the goal! The start point is blocked!')

    elif status == 2:  # End point is blocked
        stop_searching = 1
        print('There is no path to the goal! The end point is blocked!')

    if show_animation:  # Draw the border line in case of a blocked path
        info = 'There is no path to the goal! Robot and goal are split by the border shown in red \'x\'!'
        if status == 1:
            border = get_border_line(origin_closed, obstacle_coordinates)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()
        elif status == 2:
            border = get_border_line(goal_closed, obstacle_coordinates)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()

    return stop_searching, path


def searching_control(start_coor, goal_coor, boundary_coordinates, obstacle_coordinates):
    """
    Manage the searching process, start searching from two sides.
    """

    # Initialize origin node and goal node
    origin_node = Node(coor=start_coor, h_cost=heuristic_cost(start_coor, goal_coor))
    goal_node = Node(coor=goal_coor, h_cost=heuristic_cost(goal_coor, start_coor))

    # List for searching from origin to goal
    origin_open_list = [origin_node]
    origin_closed_list = []

    # List for searching from goal to origin
    goal_open_list = [goal_node]
    goal_closed_list = []

    # Initialize the target for searching from origin to goal
    target_goal_coordinates = goal_coor

    # Status: 0 (not blocked), 1 (start point blocked), 2 (end point blocked)
    status = 0  # Initial status
    path = None

    while True:
        # Searching from start to end
        origin_open_list, origin_closed_list = find_path(origin_open_list, origin_closed_list, target_goal_coordinates, boundary_coordinates)

        if not origin_open_list:  # No path condition
            status = 1  # The origin node is blocked
            draw_control(origin_closed_list, goal_closed_list, status, start_coor, goal_coor, boundary_coordinates, obstacle_coordinates)
            break

        # Update the target for searching from end to start
        target_origin_coordinates = min(origin_open_list, key=lambda x: x.f_cost).coor

        # Searching from end to start
        goal_open_list, goal_closed_list = find_path(goal_open_list, goal_closed_list, target_origin_coordinates, boundary_coordinates)

        if not goal_open_list:  # No path condition
            status = 2  # The goal is blocked
            draw_control(origin_closed_list, goal_closed_list, status, start_coor, goal_coor, boundary_coordinates, obstacle_coordinates)
            break

        # Update the target for searching from start to end
        target_goal_coordinates = min(goal_open_list, key=lambda x: x.f_cost).coor

        # Continue searching and draw the process
        stop_signal, path = draw_control(origin_closed_list, goal_closed_list, status, start_coor,
                                        goal_coor, boundary_coordinates, obstacle_coordinates)
        if stop_signal:
            break

    return path


def main(obstacle_number=1500):
    print(__file__ + ' start!')

    top_right_vertex = [60, 90]  # Top right vertex of the boundary
    bottom_left_vertex = [10, 0]  # Bottom left vertex of the boundary

    # Generate start and goal points randomly
    start_coor = generate_random_coordinates(bottom_left_vertex, top_right_vertex)
    goal_coor = generate_random_coordinates(bottom_left_vertex, top_right_vertex)

    # Generate boundary and obstacles
    boundary_and_obstacles, obstacle_coordinates = generate_boundary_and_obstacles(start_coor, goal_coor,
                                                                                top_right_vertex, bottom_left_vertex, obstacle_number)

    path = searching_control(start_coor, goal_coor, boundary_and_obstacles, obstacle_coordinates)
    if not show_animation:
        print(path)


if __name__ == '__main__':
    main(obstacle_number=1500)
