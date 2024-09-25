import heapq
from search.algorithms import State
from search.map import Map

def octile_distance(state_a, state_b):
    dx = abs(state_a.get_x() - state_b.get_x())
    dy = abs(state_a.get_y() - state_b.get_y())
    D1 = 1
    D2 = 1.5

    return D1 * (dx + dy) + (D2 - 2 * D1) * min(dx, dy)

def a_star(start, goal, gridded_map):
    OPEN = []
    CLOSED = {}

    start.set_g(0)
    start.set_cost(octile_distance(start, goal))
    heapq.heappush(OPEN, (0, start))
    CLOSED[start.state_hash()] = start.get_g()

    while OPEN:
        current_cost, current_node = heapq.heappop(OPEN)
        if current_node == goal:
            return current_node.get_g(), len(CLOSED)
        
        for child in gridded_map.successors(current_node):
            dx = abs(child.get_x() - current_node.get_x())
            dy = abs(child.get_y() - current_node.get_y())
            move_cost = gridded_map.cost(dx, dy)
            child_g = current_node.get_g() + move_cost
            h_value = octile_distance(child, goal)
            f_value = child_g + h_value

            if child.state_hash() not in CLOSED or child_g < CLOSED[child.state_hash()]:
                child.set_g(child_g)
                child.set_cost(f_value)
                CLOSED[child.state_hash()] = child_g
                heapq.heappush(OPEN, (f_value, child))

    return -1, len(CLOSED)

