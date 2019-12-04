import logging
from queue import PriorityQueue
from planner.graph import PlannerGraph
from planner.path import Path

def a_star(graph: PlannerGraph, heuristic, start, goal) -> Path:
    """
    A* search to find a minimum cost path from start to goal node on networkx.Graph

    :param graph:
    :param start: tuple, start coordinate
    :param goal: tuple, goal coordinate
    :return: path
    """

    # Find nearest nodes for start and goal
    start_node = graph.find_nearest_node(start)
    goal_node = graph.find_nearest_node(goal)

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
            for next_node in graph[current_node]:
                if next_node in visited:
                    continue

                # Compute branch and queue costs
                move_cost = graph.edges[current_node, next_node]['weight']
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
        n = goal_node
        path_cost = branch[n][0]
        path_nodes.append(goal_node)
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
