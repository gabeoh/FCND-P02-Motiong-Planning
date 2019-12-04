import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree

class DataMap:
    def __init__(self, data):
        """
        Set data grid boundaries

        :param data: NP array of obstacle data in [posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ]
        """
        self._obstacle_data = data
        self._grid = None
        self._ob_centers = []
        self._edges = []
        self._ob_polygons = []
        self._ob_kdt = None

        self.n_min = np.amin(data[:, 0] - data[:, 3])
        self.n_max = np.amax(data[:, 0] + data[:, 3])
        self.n_center_min = np.amin(data[:, 0])
        self.e_min = np.amin(data[:, 1] - data[:, 4])
        self.e_max = np.amax(data[:, 1] + data[:, 4])
        self.e_center_min = np.amin(data[:, 1])
        self.a_min = np.amin(data[:, 2] - data[:, 4])
        self.a_max = np.amax(data[:, 2] + data[:, 4])
        self.a_center_min = np.amin(data[:, 2])

        self.n_size = int(np.ceil(self.n_max - self.n_min))
        self.e_size = int(np.ceil(self.e_max - self.e_min))
        self.a_size = int(np.ceil(self.a_max - self.a_min))

    @property
    def obstacle_data(self):
        return self._obstacle_data

    @property
    def grid(self):
        return self._grid

    @property
    def graph(self):
        return self._graph

    @property
    def edges(self):
        return self._edges

    def create_grid(self, drone_altitude, safety_distance):
        """
        Create a 2-D grid representation of obstacle data at given altitude with safety margin

        :param drone_altitude:
        :param safety_distance:
        :return: self._grid
        """

        # Initialize an empty grid
        self._grid = np.zeros((self.n_size, self.e_size))
        self._ob_centers = []

        # Populate the grid with obstacles
        for i in range(self.obstacle_data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = self.obstacle_data[i, :]
            self._ob_centers.append((north, east, alt))

            # Check if the obstacle is within the safety margin of flying altitude
            alt_min = alt - d_alt - safety_distance
            alt_max = alt + d_alt + safety_distance
            if alt_min > drone_altitude or alt_max < drone_altitude:
                continue

            n_coord_min = max(0, int(np.floor(north - self.n_center_min - d_north - safety_distance)))
            n_coord_max = min(self.n_size - 1, int(np.ceil(north - self.n_center_min + d_north + safety_distance)))
            e_coord_min = max(0, int(np.floor(east - self.e_center_min - d_east - safety_distance)))
            e_coord_max = min(self.e_size - 1, int(np.ceil(east - self.e_center_min + d_east + safety_distance)))

            # Set grid cells containing obstacles to 1 - grid[n, e] = 1
            self._grid[n_coord_min:n_coord_max + 1, e_coord_min:e_coord_max + 1] = 1

        return self._grid

    def extract_obstacle_polygons(self):
        for obstacle in self._obstacle_data:
            north, east, alt, d_north, d_east, d_alt = obstacle[:]

            # Extract the 4 corners of the obstacle
            corners = [(north - d_north, east - d_east),
                       (north + d_north, east - d_east),
                       (north + d_north, east + d_east),
                       (north - d_north, east + d_east)]

            # Compute the height of the polygon
            height = alt + d_alt

            # Once you've defined corners, define polygons
            p = Polygon(corners)
            self._ob_polygons.append((p, height))
        return self._ob_polygons

    def build_obstacle_kdtree(self):
        self._ob_kdt = KDTree(self._obstacle_data[:, 0:2])

    def create_voronoi_edges(self):
        if self._grid is None or self._ob_centers is None:
            raise RuntimeError("Cannot create Voronoi edges without first creating grid")

        # create a Voronoi graph based on location of obstacle centers
        ob_centers_2d = [(ob[0], ob[1]) for ob in self._ob_centers]
        v_graph = Voronoi(ob_centers_2d)
        # voronoi_plot_2d(v_graph)

        # check each edge from graph.ridge_vertices for collision
        for v in v_graph.ridge_vertices:
            v1, v2 = v_graph.vertices[v[0]], v_graph.vertices[v[1]]

            # Convert vertex coordinates to grid index
            try:
                p1 = self.coord2grid(v1)
                p2 = self.coord2grid(v2)
            except ValueError as e:
                # print("Exclude out-of-bound vertices; {}".format(e))
                continue

            # check if edge connecting p1 and p2 are in collision using Bresenham algorithm
            in_collision = False
            cells = bresenham(p1[0], p1[1], p2[0], p2[1])
            for cell in cells:
                # Exclude if cell goes out of bound
                if cell[0] < 0 or cell[0] >= self.n_size or \
                        cell[1] < 0 or cell[1] >= self.e_size:
                    in_collision = True
                    break
                if self._grid[cell[0], cell[1]] == 1:
                    in_collision = True
                    break

            if not in_collision:
                self._edges.append(((v1[0], v1[1]), (v2[0], v2[1])))

        return self._edges

    def collides(self, point, num_nearby_polygons=10):
        point_2d = (point[0], point[1])
        if self._ob_kdt is None:
            polygons = self._ob_polygons
        else:
            poly_idxs = self._ob_kdt.query([point_2d], k=num_nearby_polygons, return_distance=False)
            polygons = [self._ob_polygons[i] for i in poly_idxs[0]]

        p = Point(point_2d)
        p_height = point[2]
        for poly, poly_height in polygons:
            if poly.contains(p) and p_height <= poly_height:
                return True
        return False

    def can_connect(self, p1, p2, num_nearby_obs):
        p1_2d, p1_height = (p1[0], p1[1]), p1[2]
        p2_2d, p2_height = (p2[0], p2[1]), p2[2]
        line = LineString([p1, p2])

        if self._ob_kdt is None:
            polygons = self._ob_polygons
        else:
            poly_idxs = self._ob_kdt.query([p1_2d, p2_2d], k=num_nearby_obs, return_distance=False)
            unique_idxs = list(set().union(poly_idxs[0], poly_idxs[1]))
            polygons = [self._ob_polygons[i] for i in unique_idxs]

        for poly, poly_height in polygons:
            if poly.crosses(line) and min(p1_height, p2_height) <= poly_height:
                return False
        return True

    def coord2grid(self, coord):
        if coord[0] < self.n_min or coord[0] > self.n_max or coord[1] < self.e_min or coord[1] > self.e_max:
            raise ValueError("The cell, {}, is out of boundary".format(coord))
        n_grid = int(round(coord[0] - self.n_min))
        e_grid = int(round(coord[1] - self.e_min))
        return (n_grid, e_grid)

    def grid2coord(self, grid):
        if grid[0] < 0 or grid[0] >= self.n_size or grid[1] < 0 or grid[1] > self.e_size:
            raise ValueError("The grid, {}, is out of boundary".format(grid))
        return (grid[0] + self.n_min, grid[1] + self.e_min)

    def print_details(self):
        print("##################################")
        print("# Datamap Details")
        print("##################################")
        print("North: ({}, {}) => {}".format(self.n_min, self.n_max, self.n_size))
        print("East: ({}, {}) => {}".format(self.e_min, self.e_max, self.e_size))
        print("Altitude: ({}, {}) => {}".format(self.a_min, self.a_max, self.a_size))

        if self.grid is None:
            print("No grid has been set")
        else:
            print("Grid Shape: {}".format(self.grid.shape))
        print("Num Obstacles: {}".format(len(self._ob_centers)))
        print("Num Edges: {}".format(len(self._edges)))

        print()

    def draw_grid(self, ax):
        if self.grid is None:
            print("No grid has been set")
            return

        x_min = int(np.floor(self.e_min))
        x_max = x_min + self.e_size
        y_min = int(np.floor(self.n_min))
        y_max = y_min + self.n_size
        ax.imshow(self.grid, cmap='Greys', origin='lower', extent=[x_min, x_max, y_min, y_max])

        ax.set_ylabel('NORTH')
        ax.set_xlabel('EAST')

    def draw_edges(self, ax):
        for edge in self._edges:
            p1 = edge[0]
            p2 = edge[1]
            ax.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
