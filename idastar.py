import sys
from collections import deque
from astar import RoutingProblem
from ways import graph, info
from ways.tools import compute_distance

r = graph.load_map_from_csv()
problem = RoutingProblem(0, 0, r)


def idastar_search(s, t):
    problem.goal = t
    problem.s_start = s
    max_speed = max(info.SPEED_RANGES[0])
    new_limit = compute_distance(r[s].lat, r[s].lon, r[t].lat, r[t].lon) / max_speed

    def dfs_l(f_limit):
        start = problem.G[problem.s_start]
        end = problem.G[problem.goal]
        max_speed = max(info.SPEED_RANGES[0])
        new_limit = sys.maxsize
        frontier = deque()
        frontier.append(Node(problem.s_start, air_dis=problem.hx(problem.s_start)))
        while frontier:
            node = frontier.pop()
            new_f = node.path_cost + node.air_dis
            if new_f > f_limit:
                new_limit = min(new_limit, new_f)
            else:
                frontier.extend(child for child in node.expand(problem))
            if problem.is_goal(node.state):
                return node.solution(), node.path_cost, compute_distance(start.lat, start.lon, end.lat,
                                                                         end.lon) / max_speed
        return None, new_limit, compute_distance(start.lat, start.lon, end.lat,
                                                 end.lon) / max_speed

    while True:
        sol, new_limit, dis = dfs_l(new_limit)
        if sol:
            return sol, new_limit, dis


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0, air_dis=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        self.air_dis = air_dis
        if parent:
            self.depth = parent.depth + 1

    def expand(self, problem):
        return ordered_set([self.child_node(problem, action) for action in problem.actions(self.state)])

    def child_node(self, problem, action):
        next_state = problem.succ(self.state, action)
        next_node = Node(next_state, self, action,
                         self.path_cost + problem.step_cost(self.state, action),
                         problem.hx(action))
        return next_node

    def solution(self):
        return [node.state for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            if node.parent is None:
                path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __repr__(self):
        return f"<{self.state}>"

    def __lt__(self, node):
        return self.state < node.state

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __ne__(self, other):
        return not (self == other)

    def __hash__(self):
        return hash(self.state)


def ordered_set(coll):
    return dict.fromkeys(coll).keys()
