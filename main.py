import heapq
import math
import osmnx as ox
import networkx as nx

# Define the Haversine distance function
def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    r = 6371  # Radius of Earth in kilometers
    return r * c

# Define the heuristic function
def heuristic(node1, node2, nodes):
    lat1, lon1 = nodes.loc[node1, ['y', 'x']]
    lat2, lon2 = nodes.loc[node2, ['y', 'x']]
    return haversine(lat1, lon1, lat2, lon2)

# A* algorithm implementation
def a_star(graph, start, goal, heuristic, nodes):
    # Priority queue
    queue = []
    heapq.heappush(queue, (0, start))
    
    # Cost maps
    g_cost = {start: 0}
    f_cost = {start: heuristic(start, goal, nodes)}
    came_from = {start: None}
    
    while queue:  # Main loop
        current_cost, current_node = heapq.heappop(queue)
        
        # Goal check
        if current_node == goal:
            path = []
            while current_node:  # Path reconstruction loop
                path.append(current_node)  # Add current node to path
                current_node = came_from[current_node]  # Move to parent node
            return path[::-1]  # Return reversed path
        
        # Explore neighbors
        for neighbor in graph.neighbors(current_node):
            tentative_g_cost = g_cost[current_node] + graph.edges[current_node, neighbor, 0]['length']
            
            if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                came_from[neighbor] = current_node
                g_cost[neighbor] = tentative_g_cost
                f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor, goal, nodes)
                heapq.heappush(queue, (f_cost[neighbor], neighbor))
    
    return None  # No path found
