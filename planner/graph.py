import numpy as np
import networkx as nx
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham
import time
import logging
from queue import PriorityQueue

from . import util
from .grid import PlannerGrid
from .path import Path

class PlannerGraph(nx.Graph):
    def __init__(self, grid: PlannerGrid):
        """
        Create graph object based on the grid

        :param grid:
        """
        super().__init__()

        t0 = time.perf_counter()
        logging.info("Creating PlannerGraph using Voronoi edges")

        edges = self.create_voronoi_edges(grid)
        for edge in edges:
            p1 = tuple(int(round(coord)) for coord in edge[0])
            p2 = tuple(int(round(coord)) for coord in edge[1])
            self.add_edge(p1, p2, weight=util.distance(p1, p2))

        elapsed_time = time.perf_counter() - t0
        logging.info("Elapsed Time (PlannerGraph): {:.3f} seconds".format(elapsed_time))

    def create_voronoi_edges(self, grid: PlannerGrid):
        edges = []

        # create a Voronoi graph based on location of obstacle centers
        ob_centers_2d = [(ob[0], ob[1]) for ob in grid.ob_centers]
        v_graph = Voronoi(ob_centers_2d)
        # voronoi_plot_2d(v_graph)

        # check each edge from graph.ridge_vertices for collision
        for v in v_graph.ridge_vertices:
            v1, v2 = v_graph.vertices[v[0]], v_graph.vertices[v[1]]

            # Convert vertex coordinates to grid index
            try:
                p1 = grid.coord2grid(v1)
                p2 = grid.coord2grid(v2)
            except ValueError as e:
                # print("Exclude out-of-bound vertices; {}".format(e))
                continue

            # check if edge connecting p1 and p2 are in collision using Bresenham algorithm
            in_collision = False
            cells = bresenham(p1[0], p1[1], p2[0], p2[1])
            for cell in cells:
                # Exclude if cell goes out of bound
                if cell[0] < 0 or cell[0] >= grid.n_size or \
                        cell[1] < 0 or cell[1] >= grid.e_size:
                    in_collision = True
                    break
                if grid.grid[cell[0], cell[1]] == 1:
                    in_collision = True
                    break

            if not in_collision:
                edges.append(((v1[0], v1[1]), (v2[0], v2[1])))

        return edges

    def a_star(self, start, goal, heuristic=util.distance) -> Path:
        """
        A* search to find a minimum cost path from start to goal node on the graph (self)

        :param start: tuple, start coordinate
        :param goal: tuple, goal coordinate
        :param heuristic:
        :return: path
        """

        # Find nearest nodes for start and goal
        start_node = self.find_nearest_node(start)
        goal_node = self.find_nearest_node(goal)

        queue = PriorityQueue()
        queue.put((0, start_node))
        visited = set(start_node)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start_node:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal_node:
                logging.debug('Found a path.')
                found = True
                break
            else:
                for next_node in self[current_node]:
                    if next_node in visited:
                        continue

                    # Compute branch and queue costs
                    move_cost = self.edges[current_node, next_node]['weight']
                    branch_cost = current_cost + move_cost
                    queue_cost = branch_cost + heuristic(next_node, goal_node)

                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

        # Retrace branch and build path
        path_nodes = []
        path_cost = 0.0
        if found:
            # retrace steps
            path_nodes.append(goal_node)
            if goal_node in branch:
                n = goal_node
                path_cost = branch[n][0]
                while branch[n][1] != start_node:
                    path_nodes.append(branch[n][1])
                    n = branch[n][1]
                path_nodes.append(branch[n][1])
        else:
            logging.warning('**********************')
            logging.warning('Failed to find a path!')
            logging.warning('**********************')

        path = Path(path_nodes[::-1], path_cost)
        return path

    def find_nearest_node(self, target):
        """
        Find a graph node that's nearest to the given target point

        :param target:
        :return:
        """
        nodes = np.array(self.nodes)

        # Find start coordinate on skeleton
        dist_array = np.linalg.norm(nodes - target, ord=2, axis=1)
        i_min_dist = np.argmin(dist_array)
        nearest_node = tuple(nodes[i_min_dist])

        return nearest_node

    def draw_edges(self, ax):
        for edge in self.edges:
            p1 = edge[0]
            p2 = edge[1]
            ax.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
