import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Define waypoints manually (example coordinates, adjust to your map)
waypoints = np.array([
    [1, 1],  # Node 0grid_snapshot
    [4, 1],  # Node 1
    [4, 4],  # Node 2
    [1, 4],  # Node 3
    [2.5, 2.5]  # Node 4 (center node)
])

# Define edges manually (only connect nodes that have a clear straight path)
# Example: (0,1), (1,2), (2,3), (3,0), and diagonals (0,4), (1,4), etc.
edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (0, 4),
    (1, 4),
    (2, 4),
    (3, 4)
]

# Create graph and add edges
G = nx.Graph()
G.add_nodes_from(range(len(waypoints)))
G.add_edges_from(edges)

# Compute adjacency matrix (1 if edge exists, else 0)
adjacency_matrix = nx.to_numpy_array(G)
print("Adjacency matrix:")
print(adjacency_matrix)

# Compute weight matrix using Euclidean distance between waypoints
weight_matrix = np.zeros_like(adjacency_matrix, dtype=float)
for i, j in G.edges():
    dist = np.linalg.norm(waypoints[i] - waypoints[j])
    weight_matrix[i, j] = dist
    weight_matrix[j, i] = dist  # undirected graph

print("\nWeight matrix:")
print(weight_matrix)

# Visualization
pos = {i: waypoints[i] for i in range(len(waypoints))}
plt.figure(figsize=(6,6))
nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=600, font_weight='bold')

# Draw edge labels (weights)
edge_labels = {(i, j): f"{weight_matrix[i,j]:.2f}" for i,j in G.edges()}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

plt.title("Graph with Waypoints and Weighted Edges")
plt.grid(True)
plt.show()
