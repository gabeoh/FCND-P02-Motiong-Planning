import argparse
import time
import msgpack
from enum import Enum, auto
import re
import utm
import logging
import numpy as np
import matplotlib.pyplot as plt

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from planner import Path, PlannerGrid, PlannerGraph

# Define control parameters
VELOCITY_THRESHOLD = 0.5
ACCEPTANCE_RADIUS = 0.5
ACCEPTANCE_RADIUS_INTERMEDIATE = 5
TAKEOFF_HEIGHT_ACCEPTANCE = 0.95

# Default goal location (home - lat0, lon0: 37.79248, -122.39745)
DEFAULT_GOAL_LATITUDE = 37.796
DEFAULT_GOAL_LONGITUDE = -122.3966

class PlannerType(Enum):
    SKIP_PLANNING=0
    GRID = 1
    GRAPH = 2

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class MotionPlanning(Drone):

    def __init__(self, connection, plot_path=False, planner_type=PlannerType.GRAPH,
                 goal_lat=DEFAULT_GOAL_LATITUDE, goal_lon=DEFAULT_GOAL_LONGITUDE):
        super().__init__(connection)
        self._plot_path = plot_path
        self._planner_type =PlannerType(planner_type)
        self._goal_latitude = goal_lat
        self._goal_longitude = goal_lon

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if self.is_takeoff_height_reached():
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if self.is_at_target_location(acceptance_radius=ACCEPTANCE_RADIUS_INTERMEDIATE):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    # When arriving at the goal, make sure that the vehicle location threshold is more strict
                    # and it is stabilized at the target location before transitioning into the landing sequence
                    if self.is_at_target_location() and self.is_hovering():
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if not self.armed and not self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        logging.info("Searching for a path to the goal at ({:.5f}, {:.5f})".format(self._goal_latitude, self._goal_longitude))
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        if self._planner_type == PlannerType.GRAPH:
            SAFETY_DISTANCE = 2

        # read lat0, lon0 from colliders into floating point values
        with open("colliders.csv", "r") as f:
            line = f.readline().rstrip()
        lat_lon0 = re.split('[\s,]+', line)
        lat0, lon0 = float(lat_lon0[1]), float(lat_lon0[3])
        home_e, home_n, zone_number, zone_letter = utm.from_latlon(lat0, lon0)
        logging.debug("CSV - lat0, lon0: {}, {}".format(lat0, lon0))
        logging.debug("Home UTM - North: {}, East: {}, Zone Number: {}, Zone Letter: {}".format(
            home_n, home_e, zone_number, zone_letter))

        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # retrieve current global position
        global_position = self.global_position[:]
        logging.debug("self.global_home (lon, lat, alt): {}".format(self.global_home))
        logging.debug("self.global_position (lon, lat, alt): {}".format(global_position))

        # convert to current local position using global_to_local()
        local_position_from_global = global_to_local(global_position, self.global_home)
        logging.debug("self.local_position (north, east, down): {}".format(self.local_position))
        logging.debug("local_position_from_global (north, east, down): {}".format(local_position_from_global))
        # TODO - Investigate the difference between self.local_position and global_to_local()

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        grid = PlannerGrid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid.print_details()

        # Define the starting point as the current position of the vehicle on the grid
        start_global = global_position
        start_local = self.local_position[:]
        start_grid = grid.coord2grid(start_local)
        logging.debug("START - Global: {}, Local: {}".format(start_global, start_local))

        # Set initial target position to the 2-D start location at target_altitude height
        self.target_position = np.array([start_local[0], start_local[1], TARGET_ALTITUDE, 0])

        # Set goal as some arbitrary position on the grid
        # Home (lat0, lon0): 37.79248, -122.39745
        goal_global = (self._goal_longitude, self._goal_latitude, TARGET_ALTITUDE)
        goal_local = global_to_local(goal_global, self.global_home)
        goal_grid = grid.coord2grid(goal_local)
        logging.debug("GOAL - Global: {}, Local: {}".format(goal_global, goal_local))

        # Grid-based path finder
        if self._planner_type == PlannerType.GRID:
            # Find path using grid-based a*
            path: Path = grid.a_star(start_grid, goal_grid)
            logging.debug(path.details)

            # Plot grid and path
            if self._plot_path:
                fig, ax = plt.subplots()
                grid.draw(ax)
                path.draw(ax)
                fig.show()

            # prune path to minimize number of waypoints
            path.prune()
            logging.debug(path.details)
        elif self._planner_type == PlannerType.GRAPH:
            # Find path using graph-based a* (Voroni edge algorithm to build the graph)
            graph = PlannerGraph(grid)
            path: Path = graph.a_star(start_local[:2], goal_local[:2])
            logging.debug(path.details)

            # Plot grid and edges
            if self._plot_path:
                fig, ax = plt.subplots()
                grid.draw(ax)
                graph.draw_edges(ax)
                ax.plot(start_local[1], start_local[0], marker='o', markersize=9, color='red')
                ax.plot(goal_local[1], goal_local[0], marker='*', markersize=9, color='red')
                path.draw(ax)
                fig.show()
        else:
            path = Path()

            # Plot the start and goal locations in the grid
            if self._plot_path:
                fig, ax = plt.subplots()
                grid.draw(ax)
                ax.plot(start_local[1], start_local[0], marker='o', markersize=9, color='red')
                ax.plot(goal_local[1], goal_local[0], marker='x', markersize=9, color='red')
                fig.show()

        if not path.is_empty():
            # Convert path noodes to waypoints
            waypoints = np.c_[path.nodes_offset, np.full(path.num_nodes, TARGET_ALTITUDE, dtype=int),
                              np.zeros(path.num_nodes, dtype=int)].tolist()
        else:
            # Handle empty path
            waypoints = np.array([[start_local[0], start_local[1], TARGET_ALTITUDE / 2, 0]],
                                 dtype=int).tolist()

        if len(waypoints) > 0:
            # Set self.waypoints
            self.waypoints = waypoints

            # send waypoints to sim (this is just for visualization of waypoints)
            self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    def is_hovering(self):
        """
        Check if the vehicle is stable/hovering at a location
        :return: True if the magnitude of the velocity vector is smaller than the threshold
        """
        return np.linalg.norm(self.local_velocity) < VELOCITY_THRESHOLD

    def is_at_target_location(self, acceptance_radius=ACCEPTANCE_RADIUS):
        """
        Check if the vehicle is within the acceptance_radius of the target location
        :param acceptance_radius:
        :return: True if the vehicle is within the acceptance radius
        """
        return np.linalg.norm(self.local_position[0:2] - self.target_position[0:2]) < acceptance_radius

    def is_takeoff_height_reached(self):
        """
        Check if takeoff target height is reached within the acceptance threshold
        :return: True if the height is reached within the threshold
        """
        return -1.0 * self.local_position[2] > TAKEOFF_HEIGHT_ACCEPTANCE * self.target_position[2]

    def is_landed(self):
        """
        Check if the vehicle is landed on the ground
        :return:
        """
        return self.global_position[2] - self.global_home[2] < 0.1 and abs(self.local_position[2]) < 0.01


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s [%(name)s]: %(message)s")
    plt.rcParams['figure.figsize'] = 14, 14

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--plot-path', help="Show path plot", action="store_true")
    parser.add_argument('--planner-type', type=int, default=2, help="1: Grid-based, 2: Graph-based (Default: 2)")
    parser.add_argument('--goal-latitude', type=float, default=DEFAULT_GOAL_LATITUDE,
                        help="The latitude of the goal location (Default: {})".format(DEFAULT_GOAL_LATITUDE))
    parser.add_argument('--goal-longitude', type=float, default=DEFAULT_GOAL_LONGITUDE,
                        help="The longitude of the goal location (Default: {})".format(DEFAULT_GOAL_LONGITUDE))
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, plot_path=args.plot_path, planner_type=args.planner_type,
                           goal_lat=args.goal_latitude, goal_lon=args.goal_longitude)
    time.sleep(1)

    drone.start()
