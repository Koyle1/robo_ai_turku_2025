import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Define adjacency matrix
adjacency_matrix = np.array([
# 0  1  2  3  4  5  6  7  8  9 10
 [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # 0 (0.0, 0.0)
 [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],  # 1 (1.5, -1.5)
 [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # 2 (1.5, 3.0)
 [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],  # 3 (-0.8, 3.0)
 [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],  # 4 (-0.5, 0.5)
 [0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],  # 5 (-2.2, 0.0)
 [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],  # 6 (-3.0, -2.0)
 [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],  # 7 (-4.2, -2.2)
 [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],  # 8 (-4.2, -0.15)
 [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1],  # 9 (-2.8, 0.5)
 [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]   #10 (-2.8, 3.0)
])

# Define waypoints (positions for each node)
waypoints = [
    (0.0, 0.0),     # 0
    (1.5, -1.5),    # 1
    (1.5, 3.0),     # 2
    (-0.8, 3.0),    # 3
    (-0.5, 0.5),    # 4
    (-2.2, 0.0),    # 5
    (-3.0, -2.0),   # 6
    (-4.2, -2.2),   # 7
    (-4.2, -0.15),  # 8
    (-2.8, 0.5),    # 9
    (-2.8, 3.0)     # 10
]

# --- Rotate all waypoints by 90 degrees counterclockwise ---
rotated_waypoints = [(-y, x) for (x, y) in waypoints]

# Create the graph
G = nx.from_numpy_array(adjacency_matrix)

# Position mapping for nodes after rotation
pos = {i: rotated_waypoints[i] for i in range(len(rotated_waypoints))}

# Draw the graph
plt.figure(figsize=(7, 5))
nx.draw(
    G, pos, with_labels=True, node_color='skyblue', node_size=800,
    font_size=10, font_weight='bold', edge_color='gray'
)

# Add coordinate labels
for i, (x, y) in pos.items():
    plt.text(x + 0.05, y + 0.05, f"{(round(x,2), round(y,2))}", fontsize=8, color='darkgreen')

plt.title("Graph Visualization Rotated 90° CCW")
plt.axis('equal')
plt.show()
