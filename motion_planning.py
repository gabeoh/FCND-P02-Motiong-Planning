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

class PlannerType(Enum):
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

    def __init__(self, connection, plot_path=False, planner_type=PlannerType.GRAPH):
        super().__init__(connection)
        self._plot_path = plot_path
        self._planner_type =PlannerType(planner_type)

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
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
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
                if ~self.armed & ~self.guided:
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
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

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
        logging.debug("self.local_position (north, east, down): {}".format(self.local_position))
        local_position = global_to_local(global_position, self.global_home)
        logging.debug("local_position (north, east, down): {}".format(local_position))
        logging.debug("self.local_position (north, east, down): {}".format(self.local_position))
        # TODO - Investigate the difference between self.local_position and global_to_local()

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        grid = PlannerGrid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid.print_details()

        # Define the starting point as the current position of the vehicle on the grid
        start_global = global_position
        start_local = local_position
        start_grid = grid.coord2grid(start_local)
        logging.debug("START - Global: {}, Local: {}".format(start_global, start_local))

        # Set goal as some arbitrary position on the grid
        # Home (lat0, lon0): 37.79248, -122.39745
        goal_global = (-122.3966, 37.796, TARGET_ALTITUDE)
        # goal_global = (-122.397, 37.793, TARGET_ALTITUDE)
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
                path.draw(ax)
                fig.show()

        # Convert path noodes to waypoints
        waypoints = np.c_[path.nodes_offset, np.full(path.num_nodes, TARGET_ALTITUDE, dtype=int),
                          np.zeros(path.num_nodes, dtype=int)].tolist()

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


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s [%(name)s]: %(message)s")
    plt.rcParams['figure.figsize'] = 14, 14

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--plot-path', help="Show path plot", action="store_true")
    parser.add_argument('--planner-type', type=int, default=2, help="1: Grid-based, 2: Graph-based")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, plot_path=args.plot_path, planner_type=args.planner_type)
    time.sleep(1)

    drone.start()
