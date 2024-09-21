import heapq
from search.algorithms import State
from search.map import Map

def dijskstra(start, goal, gridded_map):
    OPEN = []
    CLOSED = {}
    g_values = {}
    start.set_g(0)
    start.set_cost(0)
    heapq.heappush(OPEN, (start.get_cost(), start))
    g_values[start.state_hash()] = 0

    while OPEN:
        current_g, current_node = heapq.heappop(OPEN)
        if current_node == goal:
            return current_g, len(CLOSED)

        CLOSED[current_node.state_hash()] = current_node

        for child in gridded_map.successors(current_node):
            child_g = current_g + child.get_g()
            if child.state_hash() not in CLOSED:
                if child.state_hash() not in g_values or child_g < g_values[child.state_hash()]:
                    child.set_g(child_g)
                    child.set_cost(child_g)
                    g_values[child.state_hash()] = child_g
                    heapq.heappush(OPEN, (child.get_cost(), child))
    return -1, len(CLOSED)