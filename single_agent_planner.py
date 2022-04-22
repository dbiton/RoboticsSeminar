import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, my_agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = {}
    for constraint in constraints:
        agent = constraint['agent']
        loc_list = constraint['loc']
        time = constraint['timestep']
        if my_agent == agent:
            if len(loc_list) == 1:
                is_static = constraint['static']
                loc = loc_list[0]
                if is_static:
                    if loc in table:
                        table[loc] = min(time, table[loc])
                    else:
                        table[loc] = time
                else:
                    if time in table:
                        table[time].append(loc)
                    else:
                        table[time] = [loc]
            elif len(loc_list) == 2:
                if time in table:
                    table[time].append((loc_list[0], loc_list[1]))
                else:
                    table[time] = [(loc_list[0], loc_list[1])]
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    dynamic_constraint = (next_time in constraint_table and (next_loc in constraint_table[next_time] or
                                                             (curr_loc, next_loc) in constraint_table[next_time]))
    static_constraint = next_loc in constraint_table and next_time >= constraint_table[next_loc]
    if static_constraint:
        x = 3
    return dynamic_constraint or static_constraint


def is_goal_constrained(goal_loc, curr_time, constraint_table):
    if goal_loc in constraint_table and constraint_table[goal_loc]:
        # in this case there will never be a solution, technically impossible unless there
        # are two goals in the same location
        return True
    for time in constraint_table:
        if type(time) == int and time > curr_time and goal_loc in constraint_table[time]:
            return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['timestep'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def calculate_max_path_length(my_map, constraints):
    num_vertices = len(my_map)
    time_last_goal_reached = 0
    for constraint in constraints:
        # if is a location and not time
        if 'static' in constraint and constraint['static']:
            num_vertices -= 1  # discard one since it's a goal vertex which cannot be passed
            time_last_goal_reached = max(time_last_goal_reached, constraint['timestep'])
    # The longest possible path will wait until all other agents reach their goal (after which the scene will be static)
    # and then will pass through every possible vertex
    return time_last_goal_reached + num_vertices + 1

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    max_path_length = calculate_max_path_length(my_map, constraints)
    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # prune check
        if curr['timestep'] > max_path_length:
            continue
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and not is_goal_constrained(curr['loc'], curr['timestep'], constraint_table):
            return get_path(curr)
        for dir in range(5):
            if dir != 4:
                child_loc = move(curr['loc'], dir)
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'parent': curr,
                         'timestep': curr['timestep'] + 1}
            else:
                # stay in place
                child = {'loc': curr['loc'],
                         'g_val': curr['g_val'] + 1,
                         'h_val': curr['h_val'],
                         'parent': curr,
                         'timestep': curr['timestep'] + 1}

            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
