import heapq
import math
import osmnx as ox
import networkx as nx
import folium

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


#-------------------------------------------------------------------------------#
# Example Usage
# Define the place and download the street network
place_name = "San Francisco, California, USA"
G = ox.graph_from_place(place_name, network_type='drive')
nodes, edges = ox.graph_to_gdfs(G)

# Define start and end points (latitude, longitude)
start_point = (37.7749, -122.4194)  # San Francisco City Hall
end_point = (37.7989, -122.4662)    # Golden Gate Park

# Find the nearest nodes in the graph to the start and end points
start_node = ox.distance.nearest_nodes(G, X=start_point[1], Y=start_point[0])
end_node = ox.distance.nearest_nodes(G, X=end_point[1], Y=end_point[0])

# Debugging: Print the start and end nodes
print(f"Start node: {start_node}")
print(f"End node: {end_node}")

# Ensure the nodes exist in the graph
if start_node not in G.nodes:
    print(f"Error: Start node {start_node} is not in the graph")
if end_node not in G.nodes:
    print(f"Error: End node {end_node} is not in the graph")

# Run the A* algorithm to find the shortest path
route = a_star(G, start_node, end_node, heuristic, nodes)

# Print the route
print("Shortest path:", route)

# Plot the route on a map
if route:
    fig, ax = ox.plot_graph_route(G, route)
else:
    print("No path found")
# Initialize the map at the start location
m = folium.Map(location=[start_point[0], start_point[1]], zoom_start=13)

# Add start and end markers
folium.Marker(location=[start_point[0], start_point[1]], popup='Start', icon=folium.Icon(color='green')).add_to(m)
folium.Marker(location=[end_point[0], end_point[1]], popup='End', icon=folium.Icon(color='red')).add_to(m)

# Extract latitude and longitude of nodes in the route
route_coords = [(nodes.loc[node, 'y'], nodes.loc[node, 'x']) for node in route]

# Add route to the map
folium.PolyLine(route_coords, color='blue', weight=2.5, opacity=1).add_to(m)

# Save the map as an HTML file
m.save('route_map.html')

# Optionally display the map inline (if running in a Jupyter notebook)
# m

