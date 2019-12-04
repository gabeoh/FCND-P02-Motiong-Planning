import numpy as np
from enum import Enum
from queue import PriorityQueue
import logging
import time

from . import util
from .path import Path

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

class PlannerGrid:
    def __init__(self, data, drone_altitude=3, safety_distance=3):
        """
        Create a 2-D grid representation of obstacle data at given altitude with safety margin

        :param data: NP array of obstacle data in [posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ]
        :param drone_altitude:
        :param safety_distance:
        :return: self._grid
        """

        # Set grid boundaries
        self._obstacle_data = data
        self._n_min = np.amin(data[:, 0] - data[:, 3])
        self._n_max = np.amax(data[:, 0] + data[:, 3])
        self._n_center_min = np.amin(data[:, 0])
        self._e_min = np.amin(data[:, 1] - data[:, 4])
        self._e_max = np.amax(data[:, 1] + data[:, 4])
        self._e_center_min = np.amin(data[:, 1])
        self._a_min = np.amin(data[:, 2] - data[:, 4])
        self._a_max = np.amax(data[:, 2] + data[:, 4])
        self._a_center_min = np.amin(data[:, 2])

        self._n_size = int(np.ceil(self._n_max - self._n_min))
        self._e_size = int(np.ceil(self._e_max - self._e_min))
        self._a_size = int(np.ceil(self._a_max - self._a_min))

        self._n_offset = int(np.floor(self._n_min))
        self._e_offset = int(np.floor(self._e_min))


        # Initialize an empty grid
        self._grid = np.zeros((self._n_size, self._e_size))
        self._ob_centers = []

        # Populate the grid with obstacles
        for i in range(self._obstacle_data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = self._obstacle_data[i, :]
            self._ob_centers.append((north, east, alt))

            # Check if the obstacle is within the safety margin of flying altitude
            alt_min = alt - d_alt - safety_distance
            alt_max = alt + d_alt + safety_distance
            if alt_min > drone_altitude or alt_max < drone_altitude:
                continue

            n_coord_min = max(0, int(np.floor(north - self._n_center_min - d_north - safety_distance)))
            n_coord_max = min(self._n_size - 1, int(np.ceil(north - self._n_center_min + d_north + safety_distance)))
            e_coord_min = max(0, int(np.floor(east - self._e_center_min - d_east - safety_distance)))
            e_coord_max = min(self._e_size - 1, int(np.ceil(east - self._e_center_min + d_east + safety_distance)))

            # Set grid cells containing obstacles to 1 - grid[n, e] = 1
            self._grid[n_coord_min:n_coord_max + 1, e_coord_min:e_coord_max + 1] = 1

    @property
    def obstacle_data(self):
        return self._obstacle_data

    @property
    def grid(self):
        return self._grid

    @property
    def ob_centers(self):
        return self._ob_centers

    @property
    def n_offset(self):
        return self._n_offset

    @property
    def e_offset(self):
        return self._e_offset

    @property
    def n_size(self):
        return self._n_size

    @property
    def e_size(self):
        return self._e_size

    def coord2grid(self, coord):
        if coord[0] < self._n_min or coord[0] > self._n_max or coord[1] < self._e_min or coord[1] > self._e_max:
            raise ValueError("The cell, {}, is out of boundary".format(coord))
        n_grid = int(round(coord[0] - self._n_offset))
        e_grid = int(round(coord[1] - self._e_offset))
        return (n_grid, e_grid)

    def grid2coord(self, grid):
        if grid[0] < 0 or grid[0] >= self._n_size or grid[1] < 0 or grid[1] > self._e_size:
            raise ValueError("The grid, {}, is out of boundary".format(grid))
        return (grid[0] + self._n_offset, grid[1] + self._e_offset)

    def print_details(self):
        print("##################################")
        print("# Grid Details")
        print("##################################")
        print("North: ({}, {}) => Offset: {}, Size: {}".format(self._n_min, self._n_max, self._n_offset, self._n_size))
        print("East: ({}, {}) => Offset: {}, Size: {}".format(self._e_min, self._e_max, self._e_offset, self._e_size))
        print("Altitude: ({}, {}) => Size: {}".format(self._a_min, self._a_max, self._a_size))
        print("Grid Shape: {}".format(self.grid.shape))
        print("Num Obstacles: {}".format(len(self._ob_centers)))
        print()

    def draw(self, ax):
        x_min = self._e_offset
        x_max = x_min + self._e_size
        y_min = self._n_offset
        y_max = y_min + self._n_size
        ax.imshow(self.grid, cmap='Greys', origin='lower', extent=[x_min, x_max, y_min, y_max])

        ax.set_ylabel('NORTH')
        ax.set_xlabel('EAST')

    def a_star(self, start, goal, heuristic=util.distance) -> Path:
        """
        A* search to find a minimum cost path from start to goal node on self._grid

        :param start: tuple, start grid coordinate
        :param goal: tuple, goal grid coordinate
        :param heuristic:
        :return:
        """
        t0 = time.perf_counter()
        logging.info("Running a* on a grid")
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                logging.debug('Found a path.')
                found = True
                break
            else:
                for action in self.valid_actions(current_node):
                    # Get the next node based on the action
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    if next_node in visited:
                        continue

                    # Compute branch and queue costs
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + heuristic(next_node, goal)

                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

        # Retrace branch and build path
        path_nodes = []
        path_cost = 0.0
        if found:
            # retrace steps
            path_nodes.append(goal)
            if goal in branch:
                n = goal
                path_cost = branch[n][0]
                while branch[n][1] != start:
                    path_nodes.append(branch[n][1])
                    n = branch[n][1]
                path_nodes.append(branch[n][1])
        else:
            logging.warning('**********************')
            logging.warning('Failed to find a path!')
            logging.warning('**********************')

        path = Path(path_nodes[::-1], path_cost, (self._n_offset, self._e_offset))
        elapsed_time = time.perf_counter() - t0
        logging.info("Elapsed Time (a*): {:.3f} seconds".format(elapsed_time))
        return path

    def valid_actions(self, current_node):
        """
        Returns a list of valid actions given a grid and current node.
        """
        valid = list(Action)
        n, m = self._grid.shape[0] - 1, self._grid.shape[1] - 1
        x, y = current_node

        # Check if the node is off the grid or it's on an obstacle
        if x - 1 < 0 or self._grid[x - 1, y] == 1:
            valid.remove(Action.UP)
        if x + 1 > n or self._grid[x + 1, y] == 1:
            valid.remove(Action.DOWN)
        if y - 1 < 0 or self._grid[x, y - 1] == 1:
            valid.remove(Action.LEFT)
        if y + 1 > m or self._grid[x, y + 1] == 1:
            valid.remove(Action.RIGHT)
        if (x - 1 < 0 or y - 1 < 0) or self._grid[x - 1, y - 1] == 1:
            valid.remove(Action.NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m) or self._grid[x - 1, y + 1] == 1:
            valid.remove(Action.NORTH_EAST)
        if (x + 1 > n or y - 1 < 0) or self._grid[x + 1, y - 1] == 1:
            valid.remove(Action.SOUTH_WEST)
        if (x + 1 > n or y + 1 > m) or self._grid[x + 1, y + 1] == 1:
            valid.remove(Action.SOUTH_EAST)

        return valid
