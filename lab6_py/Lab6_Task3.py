import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from queue import PriorityQueue

grid_size = 100  

# Define waypoints in ROS coordinates
waypoints = [
    (-1.5, 1.8), (-1.8, 0.5), (-1.8, -0.8), (-1.5, -1.8),
    (0.0, 2.0), (0.8, 1.8), (1.8, 0.8), (1.8, -0.8),
    (1.5, -1.8), (0.0, -2.0)
]

# Load grid from CSV file
grid_matrix = np.loadtxt('grid_map.csv', delimiter=',', dtype=int)
grid_size = grid_matrix.shape[0]
print(f"Loaded grid size: {grid_size}x{grid_size}")

# Treat -1 (unknown) as blocked (1)
grid_matrix[grid_matrix == -1] = 1

# Convert ROS coordinates to grid indices
def ros_to_grid_coordinates(x, y, grid_size=100, map_resolution=0.1, origin_offset=50):
    j = round((x / map_resolution) + origin_offset)
    i = round(origin_offset - (y / map_resolution))
    # Ensure within grid bounds
    i = max(0, min(grid_size-1, i))
    j = max(0, min(grid_size-1, j))
    return i * grid_size + j

# Convert waypoints to node IDs
waypoint_nodes = [ros_to_grid_coordinates(x, y, grid_size) for x, y in waypoints]

# Start and end nodes from waypoints
start_waypoint_index = 0
end_waypoint_index = 8
start_node = waypoint_nodes[start_waypoint_index]
end_node = waypoint_nodes[end_waypoint_index]

print(f"Start node: {start_node}, End node: {end_node}")

adjacency_matrix = np.array([
    [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],  # Waypoint 0
    [1, 0, 1, 1, 0, 1, 1, 0, 0, 1],  # Waypoint 1
    [1, 1, 0, 1, 1, 0, 0, 1, 0, 1],  # Waypoint 2
    [1, 1, 1, 0, 0, 0, 0, 0, 1, 1],  # Waypoint 3
    [1, 0, 1, 0, 0, 1, 1, 1, 0, 0],  # Waypoint 4
    [1, 1, 0, 0, 1, 0, 1, 1, 0, 1],  # Waypoint 5
    [0, 1, 0, 0, 1, 1, 0, 1, 1, 1],  # Waypoint 6
    [0, 0, 1, 0, 1, 1, 1, 0, 1, 1],  # Waypoint 7
    [0, 0, 0, 1, 0, 0, 1, 1, 0, 1],  # Waypoint 8
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],  # Waypoint 9
], dtype=int)

# Pre-defined weight matrix (Euclidean distances between connected waypoints)
weights = np.full((len(waypoints), len(waypoints)), np.inf)
for i in range(len(waypoints)):
    for j in range(len(waypoints)):
        if adjacency_matrix[i, j] == 1:
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[j]
            distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            weights[i, j] = distance

# Get diagonal line points between two points (simple version)
def get_diagonal_line_points(x0, y0, x1, y1):
    points = []
    # Simple interpolation for diagonal lines
    steps = max(abs(x1 - x0), abs(y1 - y0))
    if steps == 0:
        return [(x0, y0)]
    
    for t in range(steps + 1):
        x = int(x0 + (x1 - x0) * t / steps)
        y = int(y0 + (y1 - y0) * t / steps)
        points.append((x, y))
    return points

# A* algorithm on waypoints graph
def heuristic(node1, node2):
    x1, y1 = waypoints[node1]
    x2, y2 = waypoints[node2]
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def run_astar(start_idx = None, goal_idx = None):
    if start_idx is None:
        start_idx = start_waypoint_index
    if goal_idx is None:
        goal_idx = end_waypoint_index

    visited = set()
    q = PriorityQueue()
    q.put((0 + heuristic(start_idx, goal_idx), 0, start_idx, [start_idx]))
    visited.add(start_idx)

    while not q.empty():
        f_cost, current_cost, current_waypoint, current_path = q.get()

        if current_waypoint == goal_idx:
            ros_path = [waypoints[i] for i in current_path]
            return ros_path

        for neighbor_id, is_a_neighbor in enumerate(adjacency_matrix[current_waypoint]):
            if is_a_neighbor == 1:
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    new_cost = current_cost + weights[current_waypoint][neighbor_id]
                    f_cost = new_cost + heuristic(neighbor_id, goal_idx)
                    q.put((f_cost, new_cost, neighbor_id, current_path + [neighbor_id]))
    
    return None

# Hauptprogramm
if __name__ == "__main__":
    ros_path, cost = run_astar(start_waypoint_index, end_waypoint_index)
    
    if ros_path:
        print("We have finished!")
        waypoint_indices = [waypoints.index(wp) for wp in ros_path]
        print("The waypoint path is: {}".format(waypoint_indices))
        print("The total cost is: {}".format(cost))
        print("ROS2 Path Coordinates:")
        print(f"ros_path = {ros_path}")
        
        # Visualize the found path
        path_grid = grid_matrix.copy()

        # Mark all waypoints
        for i, node in enumerate(waypoint_nodes):
            waypoint_i = node // grid_size
            waypoint_j = node % grid_size
            if i == start_waypoint_index:
                path_grid[waypoint_i, waypoint_j] = 3  # Start
            elif i == waypoint_indices[-1]:
                path_grid[waypoint_i, waypoint_j] = 4  # End
            else:
                path_grid[waypoint_i, waypoint_j] = 5  # Other waypoints

        # Mark paths between waypoints
        for k in range(len(waypoint_indices) - 1):
            node1 = waypoint_nodes[waypoint_indices[k]]
            node2 = waypoint_nodes[waypoint_indices[k + 1]]
            line_points = get_diagonal_line_points(node1 // grid_size, node1 % grid_size, 
                                                 node2 // grid_size, node2 % grid_size)
            for i, j in line_points:
                if 0 <= i < grid_size and 0 <= j < grid_size:
                    if path_grid[i, j] == 0:
                        path_grid[i, j] = 2

        # Plot - AUSSERHALB der Schleife!
        plt.figure(figsize=(10, 10))
        colors = ['grey', 'white', 'blue', 'green', 'red', 'orange']
        cmap = plt.cm.colors.ListedColormap(colors)

        x_ticks = np.arange(0, grid_size, 10)
        y_ticks = np.arange(0, grid_size, 10)
        x_labels = [f"{(j-50)*0.1:.1f}" for j in x_ticks]
        y_labels = [f"{(50-i)*0.1:.1f}" for i in y_ticks]

        plt.imshow(path_grid, cmap=cmap, vmin=0, vmax=5)
        plt.xticks(x_ticks, x_labels)
        plt.yticks(y_ticks, y_labels)
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('A* Path Finding Result - ROS Coordinates\nGrey=Free, White=Blocked, Green=Start, Red=End, Orange=Waypoints')
        plt.grid(True, alpha=0.3)

        # Create and plot NetworkX graph
        plt.figure(figsize=(10, 8))
        G = nx.Graph()
        
        for i, (x, y) in enumerate(waypoints):
            G.add_node(i, pos=(x, y))
        
        for i in range(len(waypoints)):
            for j in range(i + 1, len(waypoints)):
                if adjacency_matrix[i, j] == 1:
                    G.add_edge(i, j, weight=weights[i, j])
        
        pos = nx.get_node_attributes(G, 'pos')
        
        nx.draw(G, pos, with_labels=True, node_color='lightblue', 
                node_size=500, font_size=10, font_weight='bold')
        
        edge_labels = {(i, j): f'{weights[i, j]:.1f}' for i, j in G.edges()}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
        
        plt.title('Waypoint Graph Visualization')
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.show()
        
    else:
        print("No path found!")