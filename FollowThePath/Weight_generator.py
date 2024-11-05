import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.interpolate import interp1d
from Point_generator import generate_bouys_graph_weights, plot_grid_points_with_weights_and_rows, generate_grid_and_weights, find_start_end_points
from networkx.algorithms.shortest_paths import astar_path

import numpy as np
import networkx as nx

def create_graph(grid_points, weights, spacing=1, tolerance=.001):
    """
    Create a graph where each grid point is connected to its adjacent grid points,
    including diagonal connections, with weights more influenced by distance.
    Edges are added only if neighbors are within a certain tolerance of an existing node.

    Parameters:
    - grid_points (numpy.ndarray): Array of shape (num_grid_points, 2) containing the grid points.
    - weights (numpy.ndarray): Array of weights for the grid points.
    - spacing (float): The distance between adjacent grid points.
    - tolerance (float): Tolerance within which to connect a neighbor to an existing node.

    Returns:
    - G (networkx.Graph): Graph object with nodes as grid points and edges as connections between adjacent points.
    """
    G = nx.MultiDiGraph()
    G.add_nodes_from([tuple(point) for point in grid_points])

    # Create a mapping from point coordinates to index
    point_to_index = {tuple(point): idx for idx, point in enumerate(grid_points)}

    def is_within_tolerance(node, candidate, tolerance):
        return np.linalg.norm(np.array(node) - np.array(candidate)) <= tolerance

    for i in range(len(grid_points)):
        x = grid_points[i][0]
        y = grid_points[i][1]
        # Check for adjacent points (including diagonals)
        for dx, dy in [(-spacing, 0), (spacing, 0), (0, -spacing), (0, spacing),
                       (-spacing, -spacing), (-spacing, spacing), (spacing, -spacing), (spacing, spacing)]:
            neighbor = (np.round((x + dx), 3), np.round((y + dy),3))
            if neighbor in point_to_index:
                if any(is_within_tolerance(neighbor, existing_node, tolerance) 
                       for existing_node in point_to_index):
                    distance = np.linalg.norm(np.array([x, y]) - np.array(neighbor))  # Euclidean distance
                    weight = weights[i]
                    G.add_edge(neighbor, (x, y), weight=weight)
            else:
                # print(f"Missing node {neighbor}")
                pass

    return G


def heuristic(node1, node2):
    """
    Heuristic function for A* algorithm. Here, we use a weighted Euclidean distance.
    """
    return (np.linalg.norm(np.array(node1) - np.array(node2)))*.75  # Increased weight effect

def plot_graph_with_obstacles(grid_points, graph, blue_row, red_row, obstacles, weights=None, path=None, start_point=None, end_point=None, spacing=0.5):
    """
    Plot the directed graph of grid points with connections to adjacent points, including obstacles, start and end points, path, and edge weights.
    The function also includes arrows to indicate the direction of the edges, with arrow color based on the edge weights.
    """
    pos = {node: node for node in graph.nodes()}
    plt.figure(figsize=(12, 10))
    
    # Plot the obstacle points (row1 and row2)
    plt.scatter(obstacles[:, 0], obstacles[:, 1], color='black', s=100, label='Obstacle Points', zorder=4)
    plt.scatter(blue_row[:, 0], blue_row[:, 1], color='blue', s=100, label='blue Bouys', zorder=4)
    plt.scatter(red_row[:, 0], red_row[:, 1], color='red', s=100, label='red Bouys', zorder=4)

    # Extract node positions and colors for plotting
    nodes = list(graph.nodes())
    x = [pos[node][0] for node in nodes]
    y = [pos[node][1] for node in nodes]

    # Plot the grid points
    if weights is not None:
        # Apply an exponential transformation to weights for better visualization of node colors
        transformed_weights = weights
        
        # Plot nodes with colors based on transformed weights
        scatter = plt.scatter(x, y, c=transformed_weights, cmap='viridis', s=10, edgecolors='w', zorder=2)
        plt.colorbar(scatter, label='Weight (Exponential)')
    else:
        plt.scatter(x, y, color='black', s=50, edgecolors='w', zorder=2)
    
    # Draw directed edges with arrows colored based on the edge weights
    # for u, v, data in graph.edges(data=True):
    #     weight = data.get('weight', 1)  # Get the weight of the edge, default to 1 if no weight
    #     color = plt.cm.viridis(np.exp(weight) / np.exp(np.max(weights)))  # Normalize and color the edges based on the weight
        
    #     # Plot each edge with color and an arrow head
    #     nx.draw_networkx_edges(
    #         graph, pos,
    #         edgelist=[(u, v)],
    #         arrowstyle='-|>', arrowsize=10, edge_color=[color],
    #         connectionstyle='arc3,rad=0.1'
    #     )

    # Highlight the path if provided
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        plt.plot(path_x, path_y, color='black', linestyle='-', linewidth=4, marker='o', markersize=8, label='A* Path', alpha=0.8, zorder=3)
    
    # Highlight start point in green and end point in yellow
    if start_point != None:
        plt.scatter(start_point[0], start_point[1], color='blue', s=200, label='Start Point', zorder=5)
    if end_point != None:
        plt.scatter(end_point[0], end_point[1], color='blue', s=200, label='End Point', zorder=5)

    plt.title('Directed Graph of Grid Points with Obstacles and Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

    # Set equal scaling on both axes to ensure correct spacing
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()



# Generate example rows for demonstration
num_points = 2
start = 0
end = 10
noise_level = 5.0
min_distance = 10.0
num_random_points = 50

# Generate grid points and calculate weights
blue_row, red_row, obs, gridpoints, weights, start_point, end_point = generate_bouys_graph_weights(2,0,10,5,4,0,1,1.5)

blue_row = np.array([[2,4]])
red_row = np.array([[-5,3]])

gridpoints, weights = generate_grid_and_weights(blue_row=blue_row, red_row=red_row, obs=np.empty([0,2]),spacing=.5)
print(len(gridpoints))
start_point, end_point = find_start_end_points(grid_points=gridpoints, blue_row=blue_row,red_row=red_row)
if (start_point.all == None or end_point.all == None):
    print("No start/endpoint")
    plot_grid_points_with_weights_and_rows(gridpoints, blue_row, red_row, obs, start_point, end_point)
    

# Create and plot the graph
graph = create_graph(gridpoints, weights, spacing=.5)
obstacles = np.vstack((blue_row, red_row, obs))
# plot_graph_with_obstacles(gridpoints, graph, obstacles, weights, path=None, start_point=start_point, end_point=end_point)

start_node = (np.float64(start_point[0]), np.float64(start_point[1]))
end_node = (np.float64(end_point[0]), np.float64(end_point[1]))

# Now run the A* pathfinding
try:
    path = astar_path(graph, start_node, end_node, weight='weight', heuristic=heuristic)
except nx.NetworkXNoPath:
    path = []

# Plot the graph and the path
plot_graph_with_obstacles(gridpoints, graph, blue_row, red_row, obs, weights, path, start_node, end_node)

