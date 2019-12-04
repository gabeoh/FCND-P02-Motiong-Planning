import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

# Data mapping
from . import util
from .datamap import DataMap
from .graph import PlannerGraph
from .search import a_star
from .packet import Packet2d, Packet3d
from .sampler import Sampler
from .path import Path


class Planner:
    def __init__(self, obstacle_file, flight_altitude=3, safety_distance=3):
        self._obstacle_file = obstacle_file
        self._flight_altitude = flight_altitude
        self._safety_distance = safety_distance
        self._datamap = None
        self._global_plan = None
        self._local_plans = []

    def plan_all(self, start, goal, current_locations):
        # Build datamap
        self.build_datamap()
        self._datamap.print_details()

        # Generate coarse 2-D global plan
        self._global_plan = GlobalPlan(self._datamap.edges, start, goal)
        self._global_plan.print_details()

        # Generate local plan around current location
        for current_location in current_locations:
            local_plan = (LocalPlan(current_location, self._datamap, self._global_plan._path))
            local_plan.print_details()
            self._local_plans.append(local_plan)

        self.draw_local_plan()

    def build_datamap(self):
        """
        Build datamap based on self._obstacle_file

        :return:
        """

        # Load data from obstacle file
        data = np.loadtxt(self._obstacle_file, delimiter=',', dtype='Float64', skiprows=2)
        self._datamap = DataMap(data)
        self._datamap.create_grid(self._flight_altitude, self._safety_distance)
        self._datamap.create_voronoi_edges()
        self._datamap.extract_obstacle_polygons()
        self._datamap.build_obstacle_kdtree()

    def draw_datamap(self):
        # Plot - figure and axes
        fig, ax = plt.subplots()

        # Draw grid and edges
        self._datamap.draw_grid(ax)
        self._datamap.draw_edges(ax)
        fig.show()

    def draw_global_plan(self):
        # Plot - figure and axes
        fig, ax = plt.subplots()

        # Draw grid and edges
        self._datamap.draw_grid(ax)
        self._datamap.draw_edges(ax)

        # Draw global plan
        self._global_plan.draw(ax)
        fig.show()

    def draw_local_plan(self):
        # Plot - figure and axes
        fig, ax = plt.subplots()

        # Draw grid and edges
        self._datamap.draw_grid(ax)
        self._datamap.draw_edges(ax)

        # Draw global plan
        self._global_plan.draw(ax)

        # Draw local plan
        for local_plan in self._local_plans:
            local_plan.draw(ax)

            # Draw local path in 3D
            local_plan.draw_local_path_3d()

        fig.show()

class GlobalPlan:
    def __init__(self, edges, start, goal):
        self._start = start
        self._goal = goal
        self._graph = PlannerGraph(edges)
        self._path: Path = a_star(self._graph, util.distance, start, goal)

    @property
    def path(self):
        return self._path

    def print_details(self):
        print("##################################")
        print("# Global Plan")
        print("##################################")
        print("Start-Goal: {} => {}".format(self._start, self._goal))
        print(self._path.details)
        print()

    def draw(self, ax):
        # Draw actual start and goal
        plt.plot(self._start[1], self._start[0], 'ro', markersize=9)
        plt.plot(self._goal[1], self._goal[0], 'r*', markersize=9)

        # Draw path
        self._path.draw(ax)

class LocalPlan():
    def __init__(self, current_location, datamap: DataMap, global_path: Path):
        self._current_location = current_location
        self._pack2d = Packet2d(current_location)
        self._local_goal_2d = self._pack2d.find_local_goal(global_path)
        self._local_goal_3d = (self._local_goal_2d[0], self._local_goal_2d[1], current_location[2])

        self._pack3d = Packet3d(current_location)
        self._sampler = Sampler(datamap, self._pack3d)
        self._sampler.sample()
        self._graph = self.build_graph_from_samples(self._sampler.samples, datamap)
        self._path = a_star(self._graph, util.distance, current_location, self._local_goal_3d)

    def build_graph_from_samples(self, samples, datamap, num_nearby_samples=10, num_nearby_obs=10) -> PlannerGraph:
        sample_graph = PlannerGraph()
        sample_graph.add_nodes_from(samples)
        kdt_samples = KDTree(samples)
        neighbor_idxs_all = kdt_samples.query(samples, k=num_nearby_samples, return_distance=False)
        for i in range(len(samples)):
            sample = samples[i]
            neighbor_idxs = neighbor_idxs_all[i]
            for ni in neighbor_idxs:
                if ni == i:
                    continue
                neighbor = samples[ni]
                if datamap.can_connect(sample, neighbor, num_nearby_obs):
                    sample_graph.add_weighted_edges_from([(sample, neighbor, util.distance(sample, neighbor))])

        # Remove unconnected nodes
        unconnected_nodes = []
        for node in sample_graph.nodes:
            if sample_graph.degree(node) < 1:
                unconnected_nodes.append(node)
        sample_graph.remove_nodes_from(unconnected_nodes)

        return sample_graph


    def print_details(self):
        print("##################################")
        print("# Local Plan")
        print("##################################")
        print(self._pack2d.details)
        print(self._sampler.details)
        print(self._path.details)
        print()


    def draw(self, ax):

        # Current location
        plt.plot(self._current_location[1], self._current_location[0], color='yellow', marker='o', markersize=5)

        # Draw 2d packet
        self._pack2d.draw(ax)
        if self._local_goal_2d is not None:
            y, x = self._local_goal_2d
            ax.plot(x, y, 'r*', markersize=9)

        # Draw samples in 2D plt
        self._sampler.draw_samples(ax)

        # Draw local path
        self._path.draw(ax, color='blue', markersize=5)

    def draw_local_path_3d(self, color='blue', show_endpoints=True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        path_nodes = np.array(self._path.nodes)
        if show_endpoints:
            ax.plot([path_nodes[0, 1]], [path_nodes[0, 0]], [path_nodes[0, 2]], marker='o', markersize=9, color=color)
            ax.plot([path_nodes[-1, 1]], [path_nodes[-1, 0]], [path_nodes[-1, 2]], marker='x', markersize=9, color=color)
        ax.plot(path_nodes[:, 1], path_nodes[:, 0], path_nodes[:, 2], color=color, linewidth=3)

        ax.set_ylabel('NORTH')
        ax.set_xlabel('EAST')
        ax.set_zlabel('ALTITUDE')
        fig.show()
