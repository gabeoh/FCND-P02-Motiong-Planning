import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from . import util

class Path:
    def __init__(self, path_nodes, path_cost, offset=None):
        self._nodes = path_nodes
        self._cost = path_cost
        self._offset = offset

    def offset_grid_coordinate(self, coords):
        if self._offset is not None:
            coords = [coords[i] + self._offset[i] for i in range(min(len(self._offset), len(coords)))]
        return coords

    @property
    def nodes(self):
        return self._nodes

    @property
    def start(self):
        return self.offset_grid_coordinate(self._nodes[0])

    @property
    def goal(self):
        return self.offset_grid_coordinate(self._nodes[-1])

    @property
    def cost(self):
        return self._cost

    @property
    def num_nodes(self):
        return len(self._nodes)

    @property
    def nodes_offset(self):
        nodes = np.array(self._nodes)
        if self._offset is not None:
            nodes += self._offset
        return nodes

    @property
    def details(self):
        start = np.array(self.start)
        goal = np.array(self.goal)
        with np.printoptions(precision=2, suppress=True):
            msg = "Path: {} nodes from {} to {} (cost: {:.2f})".format(
                len(self._nodes), start, goal, self.cost)
        return msg

    def prune(self, collinearity_th=1e-6):
        """

        :param collinearity_th: collinearity threshold
        :return:
        """
        i = 0
        while i + 2 < len(self._nodes):
            points = [(p[0], p[1], 1) for p in self._nodes[i: i + 3]]
            if util.collinearity_check(points[0], points[1], points[2], collinearity_th):
                del self._nodes[i + 1]
            else:
                i += 1

    def draw(self, ax, color='green', markersize=9, show_endpoints=True):
        nodes = np.array(self._nodes)
        if self._offset is not None:
            nodes += self._offset
        if show_endpoints:
            ax.plot(self.start[1], self.start[0], marker='o', markersize=markersize, color=color)
            ax.plot(self.goal[1], self.goal[0], marker='x', markersize=markersize, color=color)
        ax.plot(nodes[:, 1], nodes[:, 0], color=color, linewidth=3)

    def draw_path_3d(self, color='blue', show_endpoints=True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        nodes = np.array(self._nodes)
        if show_endpoints:
            ax.plot([self.start[1]], [self.start[0]], [self.start[2]], marker='o', markersize=9, color=color)
            ax.plot([self.goal[1]], [self.goal[0]], [self.goal[2]], marker='x', markersize=9, color=color)
        ax.plot(nodes[:, 1], nodes[:, 0], nodes[:, 2], color=color, linewidth=3)

        ax.set_ylabel('NORTH')
        ax.set_xlabel('EAST')
        ax.set_zlabel('ALTITUDE')
        fig.show()

