import heapq
from search.algorithms import State
from search.map import Map

def dijskstra(start, goal, gridded_map):
    OPEN = []
    CLOSED = {}
    start.set_g(0)
    start.set_cost(0)

    heapq.heappush(OPEN, (start.get_g(), start))
    CLOSED[start.state_hash()] = 0

    while OPEN:
        current_g, current_node = heapq.heappop(OPEN)
        if current_node == goal:
            return current_g, len(CLOSED)
        
        for child in gridded_map.successors(current_node):
            move_cost = gridded_map.cost(abs(child.get_x() - current_node.get_x()), abs(child.get_y() - current_node.get_y()))
            next_cost = current_g + move_cost
            if child.state_hash() not in CLOSED or next_cost < CLOSED[child.state_hash()]:
                CLOSED[child.state_hash()] = next_cost
                heapq.heappush(OPEN, (next_cost, child))

    return -1, len(CLOSED)