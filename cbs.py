import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    if len(path1) > len(path2):
        path_long = path1
        path_short = path2
    else:
        path_long = path2
        path_short = path1
    len_path_short = len(path_short)

    vertex_long_prev = get_location(path_long, 0)
    vertex_short_prev = get_location(path_short, 0)
    for t in range(1, len_path_short):
        vertex_long_curr = get_location(path_long, t)
        vertex_short_curr = get_location(path_short, t)
        if vertex_short_curr == vertex_long_curr:
            return {'loc': [vertex_short_curr], 'timestep': t}
        elif vertex_long_curr == vertex_short_prev and vertex_short_curr == vertex_long_prev:
            # used this such that path1 is always the first even when path2 is longer
            return {'loc': [get_location(path1, t), get_location(path2, t)], 'timestep': t}
        vertex_long_prev = vertex_long_curr
        vertex_short_prev = vertex_short_curr

    vertex_short_last = vertex_short_prev
    for t in range(len_path_short, len(path_long)):
        vertex_long_curr = get_location(path_long, t)
        if vertex_long_curr == vertex_short_last:
            return {'loc': [vertex_long_curr], 'timestep': t}
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    return [{'agent0': i, 'agent1': j, 'loc': c['loc'], 'timestep': c['timestep']}
            for i in range(len(paths)) for j in range(i + 1, len(paths))
            if (c := detect_collision(paths[i], paths[j])) and c is not None]


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if len(collision['loc']) == 1:
        return [
            {'agent': collision['agent0'], 'loc': collision['loc'], 'timestep': collision['timestep']},
            {'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        ]
    # len(collision['loc']) == 2
    else:
        return [
            {'agent': collision['agent0'], 'loc': collision['loc'], 'timestep': collision['timestep']},
            {'agent': collision['agent1'], 'loc': collision['loc'].reverse(), 'timestep': collision['timestep']}
        ]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr = self.pop_node()
            collisions = curr['collisions']
            if len(collisions) == 0:
                self.print_results(curr)
                return curr['paths']
            collision = collisions[0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                next = {'constraints': curr['constraints'] + [constraint],
                        'paths': curr['paths']}
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                              agent, root['constraints'])
                if path is not None:
                    next['paths'][agent] = path
                    next['collisions'] = detect_collisions(next['paths'])
                    next['cost'] = get_sum_of_cost(next['paths'])
                    self.push_node(next)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
