
import heapq
import pandas as pd
from IPython.display import display, HTML
from ways import graph, info, compute_distance
from ways import tools
from queue import PriorityQueue
r= graph.load_map_from_csv()

def find_astar_rout_search(s, t):
    problem=RoutingProblem(s,t,r)
    def g(node):
        return node.path_cost


    def h(node):
        cost = (tools.compute_distance(problem.G[node.state].lat,problem.G[node.state].lon,
                                              problem.G[problem.goal].lat,problem.G[problem.goal].lon)) / 110
        return cost
    return best_first_graph_search(problem,f=lambda n: g(n)+h(n))



def pretty_print(df):
    return display( HTML( df.to_html().replace("\\n","<br>")))

def format_search_log(log,limit=100):
  columns=['node to expand', 'path', 'left in frontier', 'valid actions','is_goal']
  return pretty_print(pd.DataFrame(log, columns=columns).head(limit))

def format_search_with_costs_log(log, limit=100):
  columns=['node to expand', 'path', 'g','f','left in frontier', 'valid actions','is_goal']
  return pretty_print(pd.DataFrame(log, columns=columns).head(limit))


def ordered_set(coll):
  return dict.fromkeys(coll).keys()

def swap_tuple(a_tuple, i, j):
  l = list(a_tuple)
  l[i],l[j] = l[j],l[i]
  return tuple(l)

def load_routes(routes, symmetric=True):
  def insert(frm,to,cost):
    if frm in G:
      G[frm][to]=cost
    else:
      G[frm] = {to:cost}
  G = {}
  routes = routes.splitlines()
  for route in routes:
    r = route.split(',')
    insert(r[0],r[1],int(r[2]))
    if symmetric:
      insert(r[1],r[0],int(r[2]))
  return G



class PriorityQueue:

    def __init__(self, f=lambda x: x):
        self.heap = []
        self.f = f

    def append(self, item):
        heapq.heappush(self.heap, (self.f(item), item))

    def extend(self, items):
        for item in items:
            self.append(item)

    def pop(self):
        if self.heap:
            return heapq.heappop(self.heap)[1]
        else:
            raise Exception('Trying to pop from empty PriorityQueue.')

    def __len__(self):
        return len(self.heap)

    def __contains__(self, key):
        return any([item == key for _, item in self.heap])

    def __getitem__(self, key):
        for value, item in self.heap:
            if item == key:
                return value
        raise KeyError(str(key) + " is not in the priority queue")

    def __delitem__(self, key):
        try:
            del self.heap[[item == key for _, item in self.heap].index(True)]
        except ValueError:
            raise KeyError(str(key) + " is not in the priority queue")
        heapq.heapify(self.heap)
    def __repr__(self):
      return str(self.heap)


class RoutingProblem:

    def __init__(self, s_start, goal, _g):
        self.s_start = s_start
        self.goal = goal
        self.G = _g

    def actions(self, s):
        list_jun = []
        for l in self.G[s].links:
            list_jun.append(l.target)
        return list_jun

    def succ(self, s, a):
        if a in self.actions(s):
            return a
        raise ValueError(f'No route from {s} to {a}')

    def is_goal(self, s):
        return s == self.goal

    def step_cost(self, s, a):
        for link in self.G[s].links:
            if link.target == a:
                current_link = link
                max_speed = max(info.SPEED_RANGES[current_link.highway_type])
                cost = (current_link.distance / 1000) / max_speed
                return cost

    def hx(self, a):
        g = self.G[self.goal]
        curr = self.G[a]
        max_speed = max(info.SPEED_RANGES[0])
        cost = compute_distance(curr.lat, curr.lon, g.lat, g.lon) / max_speed
        return cost

    def dis(self,a):
        g = self.G[self.goal]
        curr = self.G[a]
        return compute_distance(curr.lat, curr.lon, g.lat, g.lon)


    @staticmethod
    def state_str(s):
        return s

    def __repr__(self):
        return {'s_start': self.s_start, 'goal': self.goal, graph: 'self.G'}



class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def expand(self, problem):
        return ordered_set([self.child_node(problem, action)
                            for action in problem.actions(self.state)])

    def child_node(self, problem, action):
        next_state = problem.succ(self.state, action)
        next_node = Node(next_state, self, action,
                         self.path_cost + problem.step_cost(self.state, action))
        return next_node

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
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



def best_first_graph_search(problem, f):
    node = Node(problem.s_start)
    frontier = PriorityQueue(f)  # Priority Queue
    frontier.append(node)
    closed_list = set()
    cost= problem.hx(problem.s_start)
    while frontier:
        if len(closed_list) % 1000 == 0:
            print(f'size of closed list:{len(closed_list)}')
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node.solution(), problem.hx(problem.s_start), node.path_cost
        closed_list.add(node.state)
        for child in node.expand(problem):
            if child.state not in closed_list and child not in frontier:
                frontier.append(child)
            elif child in frontier and f(child) < frontier[child]:
                del frontier[child]
                frontier.append(child)
    return

def uniform_cost_search(problem):
  def g(node):
    return node.path_cost
    return best_first_graph_search(problem, f=g)


