import logging
from shapely.geometry import Polygon, LineString

from .path import Path

class Packet2d:
    def __init__(self, curr_loc, size_n=40, size_e=40):
        self.p_size_n = size_n
        self.p_size_e = size_e
        self.n_min = curr_loc[0] - self.p_size_n / 2
        self.n_max = self.n_min + self.p_size_n
        self.e_min = curr_loc[1] - self.p_size_e / 2
        self.e_max = self.e_min + self.p_size_e

        self.packet_poly = Polygon([(self.n_min, self.e_min), (self.n_min, self.e_max),
                                    (self.n_max, self.e_max), (self.n_max, self.e_min)])

    def find_local_goal(self, path: Path, i_start=0):
        crossing_lines = []
        for i in range(i_start, len(path.nodes) - 1):
            path_segment = LineString(path.nodes[i:i + 2])
            if path_segment.crosses(self.packet_poly.exterior):
                crossing_lines.append(path_segment)

        if len(crossing_lines) < 1:
            logging.warning("Cannot find local goal")
            return

        # The last crossing on the path is the closest to the final goal
        last_crossing = crossing_lines[-1]
        intersections = last_crossing.intersection(self.packet_poly.exterior)

        # TODO - Handle multiple crossings on a line
        local_goal = intersections.coords[0]
        return local_goal

    @property
    def details(self):
        msg = "Packet2d: Rectangle corners: ({}, {}) - ({}, {})".format(
                self.n_min, self.e_min, self.n_max, self.e_max)
        return msg

    def draw(self, ax):
        y, x = self.packet_poly.exterior.xy
        ax.plot(x, y, linewidth=2, color='yellow')

class Packet3d():
    def __init__(self, curr_loc, size_n=40, size_e=40, size_alt=10):
        self.p_size_n = size_n
        self.p_size_e = size_e
        self.p_size_alt = size_alt

        self.n_min = curr_loc[0] - self.p_size_n / 2
        self.n_max = self.n_min + self.p_size_n
        self.e_min = curr_loc[1] - self.p_size_e / 2
        self.e_max = self.e_min + self.p_size_e
        self.alt_min = 0
        self.alt_max = self.alt_min + self.p_size_alt