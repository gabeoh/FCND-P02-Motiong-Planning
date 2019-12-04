import numpy as np
import networkx as nx

from . import util

class PlannerGraph(nx.Graph):
    def __init__(self, edges=[]):
        """
        Create graph object based on the provided edges

        :param edges:
        """
        super().__init__()

        for edge in edges:
            p1, p2 = edge[0], edge[1]
            self.add_edge(p1, p2, weight=util.distance(p1, p2))

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
