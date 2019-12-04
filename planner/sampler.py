import numpy as np
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
from sklearn.neighbors import KDTree
from planner.datamap import DataMap
from planner.packet import Packet3d
from planner.graph import PlannerGraph
from  matplotlib.axes._subplots import Axes
from planner.util import distance

class Sampler:
    def __init__(self, datamap: DataMap, pack3d: Packet3d):
        self._datamap = datamap
        self._pack3d = pack3d
        self._samples = []

    @property
    def samples(self):
        return self._samples

    def sample(self, num_samples=200, num_nearby_obs=10):
        # First, clear previous samples
        self._samples.clear()

        # Generate sample points
        n_vals = np.random.uniform(self._pack3d.n_min, self._pack3d.n_max, num_samples)
        e_vals = np.random.uniform(self._pack3d.e_min, self._pack3d.e_max, num_samples)
        a_vals = np.random.uniform(self._pack3d.alt_min, self._pack3d.alt_max, num_samples)
        samples = list(zip(n_vals, e_vals, a_vals))

        for sample in samples:
            if not self._datamap.collides(sample, num_nearby_obs):
                self._samples.append(sample)

    @property
    def details(self):
        msg = "Num Non-Colliding Samples: {}".format(len(self._samples))
        return msg

    def draw_samples(self, ax: Axes):
        ax.scatter([s[1] for s in self._samples], [s[0] for s in self._samples], s=1, c='red')
