import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from queue import PriorityQueue

# ===== CONFIGURATION =====
grid_size = 100  # Change this value to adjust grid size

# Define waypoints in ROS coordinates
waypoints = [
    (-1.5, 1.8), (-1.8, 0.5), (-1.8, -0.8), (-1.5, -1.8),
    (0.0, 2.0), (0.8, 1.8), (1.8, 0.8), (1.8, -0.8),
    (1.5, -1.8), (0.0, -2.0)
]
# =========================

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

# Bresenham's line algorithm to get all points between two points
def get_line_points(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x, y))
    return points

# Check if direct path between two nodes is possible (no obstacles in between)
def is_direct_path_possible(node1, node2, grid_matrix, grid_size):
    i1, j1 = node1 // grid_size, node1 % grid_size
    i2, j2 = node2 // grid_size, node2 % grid_size
    
    # Use Bresenham's line algorithm to check all points along the line
    points = get_line_points(i1, j1, i2, j2)
    
    for i, j in points:
        if grid_matrix[i, j] == 1:  # If any point is blocked
            return False
    return True

# Create adjacency matrix only for waypoints
num_waypoints = len(waypoints)
adjacency_matrix = np.zeros((num_waypoints, num_waypoints), dtype=int)
weights = np.full((num_waypoints, num_waypoints), np.inf)

# Fill adjacency matrix - connect waypoints if direct path is possible
for i in range(num_waypoints):
    for j in range(num_waypoints):
        if i != j:
            node1 = waypoint_nodes[i]
            node2 = waypoint_nodes[j]
            if is_direct_path_possible(node1, node2, grid_matrix, grid_size):
                # Calculate Euclidean distance as weight
                x1, y1 = waypoints[i]
                x2, y2 = waypoints[j]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                adjacency_matrix[i, j] = 1
                weights[i, j] = distance

# Print the binary adjacency matrix from Bresenham algorithm
print("Binary adjacency matrix:")
print("adjacency_matrix = np.array([")
for i in range(num_waypoints):
    row = [str(adjacency_matrix[i, j]) for j in range(num_waypoints)]
    print("    [" + ", ".join(row) + "],  # Waypoint " + str(i))
print("], dtype=int)")

# Dijkstra algorithm on waypoints graph
visited = set()
q = PriorityQueue()
q.put((0, start_waypoint_index, [start_waypoint_index])) # Each element in the queue is (distance, waypoint_index, path_to_waypoint)
visited.add(start_waypoint_index)

while not q.empty():
    current_cost, current_waypoint, current_path = q.get()

    if current_waypoint == end_waypoint_index:
        print("We have finished!")
        print("The waypoint path is: {}".format(current_path))
        print("The total cost is: {}".format(current_cost))
        
        # Convert waypoint indices to ROS coordinates
        ros_path = [waypoints[i] for i in current_path]
        print("ROS2 Path Coordinates:")
        print(f"ros_path = {ros_path}")
        
        # Visualize the found path
        path_grid = grid_matrix.copy()

        # Mark all waypoints
        # Mark direct paths between waypoints in the path
        # Mark direct paths between waypoints in the path
        for k in range(len(current_path) - 1):
            node1 = waypoint_nodes[current_path[k]]
            node2 = waypoint_nodes[current_path[k + 1]]
            line_points = get_line_points(node1 // grid_size, node1 % grid_size, 
                                        node2 // grid_size, node2 % grid_size)
            for i, j in line_points:
                if path_grid[i, j] == 0:  # Only mark if not already marked
                    path_grid[i, j] = 2  # Use 2 to represent the path

        for i, node in enumerate(waypoint_nodes):
            waypoint_i = node // grid_size
            waypoint_j = node % grid_size
            if i == start_waypoint_index:
                path_grid[waypoint_i, waypoint_j] = 3  # Start
            elif i == end_waypoint_index:
                path_grid[waypoint_i, waypoint_j] = 4  # End
            else:
                path_grid[waypoint_i, waypoint_j] = 5  # Other waypoints

        # Plot
        plt.figure(figsize=(10, 10))
        colors = ['grey', 'white', 'blue', 'green', 'red', 'orange']  # Free, Blocked, Path, Start, End, Waypoints
        cmap = plt.cm.colors.ListedColormap(colors)

        # Create custom tick labels for ROS coordinates
        x_ticks = np.arange(0, grid_size, 10)
        y_ticks = np.arange(0, grid_size, 10)
        x_labels = [f"{(j-50)*0.1:.1f}" for j in x_ticks]
        y_labels = [f"{(50-i)*0.1:.1f}" for i in y_ticks]

        plt.imshow(path_grid, cmap=cmap, vmin=0, vmax=5)
        plt.xticks(x_ticks, x_labels)
        plt.yticks(y_ticks, y_labels)
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Waypoint Dijkstra Path Finding Result - ROS Coordinates\nGrey=Free, White=Blocked, Green=Start, Red=End, Orange=Waypoints')
        plt.grid(True, alpha=0.3)
        plt.show()
        
        break

    for neighbor_id, is_a_neighbor in enumerate(adjacency_matrix[current_waypoint]):
        if is_a_neighbor == 1:
            if neighbor_id not in visited:
                visited.add(neighbor_id)
                # Each element in the queue is (distance, waypoint_index, path_to_waypoint)
                q.put((current_cost + weights[current_waypoint][neighbor_id], neighbor_id, current_path + [neighbor_id]))