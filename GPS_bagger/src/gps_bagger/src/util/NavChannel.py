import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.interpolate import interp1d
# from Point_generator import generate_bouys_graph_weights, plot_grid_points_with_weights_and_rows
from networkx.algorithms.shortest_paths import astar_path
from util.maps import Pose

import numpy as np
import networkx as nx



def generate_points(num_points, start, end, noise_level, min_distance, num_random_points):
    """
    Generate noisy linear points for two rows and random points constrained between these rows.

    Parameters:
    - num_points (int): Number of points to generate for each row.
    - start (float): The start value for the x-coordinate of the linear points.
    - end (float): The end value for the x-coordinate of the linear points.
    - noise_level (float): The range of noise to add to each coordinate of the linear points.
    - min_distance (float): Minimum distance between adjacent noisy points in each row.
    - num_random_points (int): Number of random scatter points to generate between the two rows.

    Returns:
    - blue_row (numpy.ndarray): Array of shape (num_points, 2) containing the noisy linear points for Row 1.
    - red_row (numpy.ndarray): Array of shape (num_points, 2) containing the noisy linear points for Row 2.
    - random_points (numpy.ndarray): Array of shape (num_random_points, 2) containing the random points constrained between Row 1 and Row 2.
    """
    
    def generate_noisy_linear_points(num_points, start, end, noise_level, min_distance):
        x = np.linspace(start, end, num_points)
        y = np.linspace(start, end, num_points)

        # Add random noise
        x_noise = np.round(np.random.uniform(-noise_level, noise_level, num_points),3)
        y_noise = np.round(np.random.uniform(-noise_level, noise_level, num_points),3)

        x_noisy = x + x_noise
        y_noisy = y + y_noise

        # Ensure minimum distance between points
        points = np.column_stack((x_noisy, y_noisy))

        def distance(p1, p2):
            return np.linalg.norm(p1 - p2)

        def filter_points(points, min_distance):
            filtered_points = [points[0]]
            for point in points[1:]:
                if distance(filtered_points[-1], point) >= min_distance:
                    filtered_points.append(point)
            return np.array(filtered_points)

        return filter_points(points, min_distance)

    def scatter_random_points_between_rows(x_range, num_random_points, min_distance, blue_row, red_row):
        x_random = np.random.uniform(x_range[0], x_range[1], num_random_points)
        y_random = np.random.uniform(x_range[0], x_range[1], num_random_points)

        # Interpolate the y-values of blue_row (red line) and red_row (blue line)
        interp_blue_row = interp1d(blue_row[:, 0], blue_row[:, 1], bounds_error=False, fill_value="extrapolate")
        interp_red_row = interp1d(red_row[:, 0], red_row[:, 1], bounds_error=False, fill_value="extrapolate")

        # Get the corresponding y-values for blue_row (below) and red_row (above)
        y_blue_row = interp_blue_row(x_random)
        y_red_row = interp_red_row(x_random)

        # Ensure random points are above blue_row and below red_row
        valid_indices = (y_random > y_blue_row) & (y_random < y_red_row)
        valid_points = np.column_stack((x_random[valid_indices], y_random[valid_indices]))

        # Function to calculate distance between two points
        def distance(p1, p2):
            return np.linalg.norm(p1 - p2)

        # Filter points based on the minimum distance
        def filter_points(points, min_distance):
            if len(points) == 0:
                return points
            filtered_points = [points[0]]
            for point in points[1:]:
                if all(distance(point, filtered_point) >= min_distance for filtered_point in filtered_points):
                    filtered_points.append(point)
            return np.array(filtered_points)

        # Ensure minimum distance between valid points
        return filter_points(valid_points, min_distance)

    # Generate noisy points for the first row
    blue_row = generate_noisy_linear_points(num_points, start, end, noise_level, min_distance)

    # Generate noisy points for the second row (parallel but offset)
    red_row = generate_noisy_linear_points(num_points, start, end, noise_level, min_distance)

    # Apply an offset to the second row
    red_row[:, 1] += 7  # Offset by 20 units vertically to ensure separation

    # Define x_range for random points
    x_range = (start, end)

    # Generate random points with minimum distance constraint that are between the two rows
    random_points = scatter_random_points_between_rows(x_range, num_random_points, min_distance, blue_row, red_row)

    return blue_row, red_row, random_points

# Example weight function
def inverse_distance_weight(distance):
    return 1/ (distance)

def generate_grid_and_weights(blue_row, red_row, obs, spacing=1, exclusion_radius=0, weight_function=None):
    """
    Generate a grid of points spaced `spacing` units apart between two rows and calculate their weights,
    excluding points within `exclusion_radius` units of any obstacles.
    """
    
    def compute_distance(point1, point2):
        return np.linalg.norm(point1 - point2)

    # Check if both blue_row and red_row are empty
    if blue_row.size == 0 and red_row.size == 0:
        pass  # No points to process
        return np.array([]), np.array([])  # Return empty arrays for grid points and weights
    


    # Check if blue_row or red_row has only one entry
    if blue_row.shape[0] == 1 and red_row.shape[0] > 1:
        # Make blue_row parallel to red_row
        offset = blue_row[0, 1] - red_row[0, 1]
        interp_red_row = interp1d(red_row[:, 0], red_row[:, 1], bounds_error=False, fill_value="extrapolate")
        interp_blue_row = lambda x: interp_red_row(x) + offset

    elif red_row.shape[0] == 1 and blue_row.shape[0] > 1:
        # Make red_row parallel to blue_row
        offset = red_row[0, 1] - blue_row[0, 1]
        interp_blue_row = interp1d(blue_row[:, 0], blue_row[:, 1], bounds_error=False, fill_value="extrapolate")
        interp_red_row = lambda x: interp_blue_row(x) + offset

    elif (blue_row.shape[0] == 1 and red_row.shape[0] == 1) or ((blue_row.shape[0] == 0) ^ (red_row.shape[0] == 0)):

        # Define line from (0, 0) to the blue or red point
        if blue_row.shape[0] > 0:
            blue_point = blue_row[0]
            start_point = np.array([0, 0])
            direction_vector = blue_point - start_point
            slope = direction_vector[1] / direction_vector[0] if direction_vector[0] != 0 else float('inf')

            # Define interpolation functions with a 15m boundary above the line
            def interpolate_above_line(x, offset):
                return slope * x + offset

            interp_blue_row = lambda x: interpolate_above_line(x, 0)
            interp_red_row = lambda x: interpolate_above_line(x, 15)

        elif red_row.shape[0] > 0:
            red_point = red_row[0]
            start_point = np.array([0, 0])
            direction_vector = red_point - start_point
            slope = direction_vector[1] / direction_vector[0] if direction_vector[0] != 0 else float('inf')

            # Define interpolation functions with a 15m boundary below the line
            def interpolate_below_line(x, offset):
                return slope * x + offset

            interp_blue_row = lambda x: interpolate_below_line(x, -15)
            interp_red_row = lambda x: interpolate_below_line(x, 0)


    else:
        # Define interpolation functions for both rows as usual
        interp_blue_row = interp1d(blue_row[:, 0], blue_row[:, 1], bounds_error=False, fill_value="extrapolate")
        interp_red_row = interp1d(red_row[:, 0], red_row[:, 1], bounds_error=False, fill_value="extrapolate")

    if blue_row.shape[0] == 0:
        x_min = red_row[:, 0].min()
        y_min = red_row[:, 1].min()
        if red_row.shape[0] == 1:
            x_max = red_row[:, 0].max() + 10
            y_max = red_row[:, 1].max() + 10
        else:
            x_max = red_row[:, 0].max() 
            y_max = red_row[:, 1].max()
    elif red_row.shape[0] == 0:
        x_min = blue_row[:, 0].min()
        y_min = blue_row[:, 1].min()
        if blue_row.shape[0] == 1:
            x_max = blue_row[:, 0].max() + 10
            y_max = blue_row[:, 1].max() + 10
        else:
            x_max = blue_row[:, 0].max()
            y_max = blue_row[:, 1].max()
            
    else:
        # Define the x and y range for the grid
        x_min = min(blue_row[:, 0].min(), red_row[:, 0].min())
        x_max = max(blue_row[:, 0].max(), red_row[:, 0].max())
        y_min = min(blue_row[:, 1].min(), red_row[:, 1].min())
        y_max = max(blue_row[:, 1].max(), red_row[:, 1].max())

    # Generate x and y coordinates for the grid
    x_coords = np.arange(x_min, x_max, spacing)
    y_coords = np.arange(y_min, y_max, spacing)

    # Create a grid of x and y coordinates
    grid_x, grid_y = np.meshgrid(x_coords, y_coords)

    # Flatten the grid for processing
    grid_x = grid_x.flatten()
    grid_y = grid_y.flatten()

    # Filter points to be between the rows
    y_blue_row = interp_blue_row(grid_x)
    y_red_row = interp_red_row(grid_x)
    valid_indices = (grid_y > y_blue_row) & (grid_y < y_red_row)
    valid_grid_points = np.column_stack((grid_x[valid_indices], grid_y[valid_indices]))

    # Combine rows into a single obstacle array
    # Combine rows into a single obstacle array with dimension checks
    obstacles = []
    if blue_row.size > 0:
        obstacles.append(blue_row)
    if red_row.size > 0:
        obstacles.append(red_row)
    if obs.size > 0:
        obstacles.append(obs)

    # Stack obstacles if there are any valid rows
    if obstacles:
        obstacles = np.vstack(obstacles)
    else:
        obstacles = np.empty((0, 2))  # Create an empty array if no obstacles

    # Calculate weights for each valid grid point, excluding points within the exclusion radius
    valid_grid_points_filtered = []
    weights = []
    
    for grid_point in valid_grid_points:
        distances = np.array([compute_distance(grid_point, obstacle) for obstacle in obstacles])
        min_distance = np.min(distances)

        if min_distance <= exclusion_radius:
            continue
        if weight_function:
            weight = weight_function(min_distance)
            valid_grid_points_filtered.append(grid_point)
            weights.append(weight)

    return np.round(np.array(valid_grid_points_filtered), 3), np.array(weights)

def find_start_end_points(grid_points, blue_row, red_row):
    def find_nearest_point(grid_points, point):
        distances = np.linalg.norm(grid_points - point, axis=1)
        nearest_index = np.argmin(distances)
        return grid_points[nearest_index]

    def find_furthest_point(grid_points, reference_point):
        distances = np.linalg.norm(grid_points - reference_point, axis=1)
        furthest_index = np.argmax(distances)
        return grid_points[furthest_index]

    # Set the start point as the closest grid point to (0, 0)
    start_point = find_nearest_point(grid_points, np.array([0, 0]))

    # Combine blue_row and red_row for obstacle checking
    obs = []
    if blue_row.size > 0:
        obs.append(blue_row)
    if red_row.size > 0:
        obs.append(red_row)

    # Flatten the list of obstacles into a single array for distance calculations
    if obs:
        obs = np.vstack(obs)
    else:
        obs = np.empty((0, 2))  # Create an empty array if no obstacles

    # Check the number of entries in blue_row and red_row
    blue_count = blue_row.shape[0]
    red_count = red_row.shape[0]

    # If either blue_row or red_row has one entry or less
    if (blue_count <= 1 and red_count > 1) or (red_count <= 1 and blue_count > 1):
        # Find the furthest point from (0, 0) that is at least 4m away from any obstacles
        valid_end_points = []

        distance_between_bouys = np.linalg.norm(blue_row[0] - red_row[0])

        for point in grid_points:
            distances_to_obs = np.array([np.linalg.norm(point - obstacle) for obstacle in obs])
            if np.all(distances_to_obs >= distance_between_bouys/2):  # Check if the point is at least 4m away from all obstacles
                valid_end_points.append(point)

        if valid_end_points:
            valid_end_points = np.array(valid_end_points)
            end_point = find_furthest_point(valid_end_points, np.array([0, 0]))  # Furthest valid point
        else:
            end_point = start_point  # Fallback if no valid points
    elif ((blue_count < 1) ^ (red_count < 1)):
        # Find the furthest point from (0, 0) that is at least 4m away from any obstacles
        valid_end_points = []

        for point in grid_points:
            distances_to_obs = np.array([np.linalg.norm(point - obstacle) for obstacle in obs])
            if np.all(distances_to_obs >= 4):  # Check if the point is at least 4m away from all obstacles
                valid_end_points.append(point)

        if valid_end_points:
            valid_end_points = np.array(valid_end_points)
            end_point = find_furthest_point(valid_end_points, np.array([0, 0]))  # Furthest valid point
        else:
            end_point = start_point  # Fallback if no valid points

    else:
        # For normal case, find the end point as the nearest point to the midpoint of last points
        last_blue_point = blue_row[-1]
        last_red_point = red_row[-1]
        midpoint = (last_blue_point + last_red_point) / 2
        end_point = find_nearest_point(grid_points, midpoint)

    return start_point, end_point




def plot_grid_points_with_weights_and_rows(grid_points, weights, blue_row, red_row, obstacles, start_point, end_point):
    """
    Plot a graph of grid points with color representing their associated weights, along with two rows (blue and red),
    obstacles, and designated start and end points.

    Parameters:
    - grid_points (numpy.ndarray): Array of valid grid points between the rows.
    - weights (numpy.ndarray): Array of weights corresponding to each grid point, used to color the points.
    - blue_row (numpy.ndarray): Array of points representing the first row (plotted in blue).
    - red_row (numpy.ndarray): Array of points representing the second row (plotted in red).
    - obstacles (numpy.ndarray): Array of points representing obstacles (plotted as black dots).
    - start_point (tuple): The point designated as the starting point for pathfinding (plotted in green).
    - end_point (tuple): The point designated as the ending point for pathfinding (plotted in green).

    Functionality:
    1. **Plot Rows:** The blue row and red row points are plotted as lines with markers.
    2. **Plot Obstacles:** Obstacles are shown as large black scatter points.
    3. **Plot Grid Points:** Valid grid points are displayed as scatter points, with color corresponding to their weights (using a color map).
    4. **Start and End Points:** The start and end points are plotted in green, marking their importance in pathfinding.
    5. **Color Bar:** A color bar indicates the weight values associated with the grid points.
    6. **Labels and Legend:** Axes labels, a legend, and a grid are added for clarity.

    The plot visually represents the layout of the grid, weights, rows, obstacles, and key points for further analysis or pathfinding purposes.
    """


    norm = Normalize(vmin=min(weights), vmax=max(weights))
    cmap = cm.viridis

    plt.figure(figsize=(12, 10))

    # Plot blue_row and red_row
    plt.plot(blue_row[:, 0], blue_row[:, 1], 'b-o', label='Row 1 (blue)')
    plt.plot(red_row[:, 0], red_row[:, 1], 'r-o', label='Row 2 (red)')

    # Plot obstacles
    plt.scatter(obstacles[:, 0], obstacles[:, 1], c='black', label='Obstacles', s=100)
    plt.scatter(start_point[0], start_point[1], c='green', label='Start', s=100)
    plt.scatter(end_point[0], end_point[1], c='green', label='End', s=100)

    # Plot grid points with weights
    sc = plt.scatter(grid_points[:, 0], grid_points[:, 1], c=weights, cmap=cmap, norm=norm, s=50, edgecolors='w')
    plt.colorbar(sc, label='Weight')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Grid Points with Weights as Color and Rows')
    plt.grid(True)
    plt.legend()
    plt.show()

def generate_bouys_graph_weights(num_bouy=2, start=0, end=20, min_dis=10, noise=3, num_rand_pt=20, grid_spacing=1, exclusion_rad=1.5, weight_function=inverse_distance_weight):
    """
    Generates a graph with weighted edges based on grid points located between two rows (representing bouys), 
    incorporating obstacles and calculating weights for each grid point. It also identifies the start and end 
    points for pathfinding.

    Parameters:
    - num_bouy (int, default=2): Number of points to generate for each of the two rows representing the bouys.
    - start (float, default=0): The starting X-coordinate for generating the linear points.
    - end (float, default=20): The ending X-coordinate for generating the linear points.
    - min_dis (float, default=10): Minimum distance between adjacent noisy points in each row to ensure proper spacing.
    - noise (float, default=3): Noise level to be added to each coordinate of the linear points to introduce variability.
    - num_rand_pt (int, default=20): Number of random scatter points to generate between the two rows, constrained within the specified range.
    - grid_spacing (float, default=1): Spacing between adjacent grid points in the generated grid.
    - exclusion_rad (float, default=1.5): Radius around obstacles within which grid points are excluded.
    - weight_function (function, default=inverse_distance_weight): A function to compute weights for the grid points based on their distance from obstacles. The default function is `inverse_distance_weight`.

    Returns:
    - blue_row (numpy.ndarray): Array of shape (num_bouy, 2) containing the noisy linear points for the blue row.
    - red_row (numpy.ndarray): Array of shape (num_bouy, 2) containing the noisy linear points for the red row.
    - obs (numpy.ndarray): Array of shape (num_rand_pt, 2) containing the random points constrained between the two rows, which act as obstacles.
    - filteblue_grid_points (numpy.ndarray): Array of shape (num_filteblue_points, 2) containing the valid grid points within the specified spacing and excluding those within the exclusion radius of obstacles.
    - weights (numpy.ndarray): Array of shape (num_filteblue_points,) containing the weights associated with each grid point, computed using the provided weight function.
    - start_point (tuple): The nearest grid point to the midpoint of the line segment connecting the first points of the blue and red rows, chosen as the starting point for pathfinding.
    - end_point (tuple): The nearest grid point to the midpoint of the line segment connecting the last points of the blue and red rows, chosen as the ending point for pathfinding.

    Functionality:
    1. Generate Bouys and Obstacles: Uses `generate_points` to create two rows of points (representing bouys) and additional random points acting as obstacles.
    2. Compute Grid Points and Weights: Calls `generate_grid_and_weights` to create a grid of points between the bouys, calculates the distance-based weights for these grid points, and excludes points within the exclusion radius of any obstacles.
    3. Determine Start and End Points: Finds the start and end points by locating the grid points nearest to the midpoints of the line segments connecting the first and last points of the blue and red rows.
    4. Return Results: Returns all generated data, including bouys, obstacles, grid points, weights, and selected start and end points.
    """
    blue_row, red_row, obs = generate_points(num_bouy, start, end, noise, min_dis, num_rand_pt)
    blue_row = blue_row[:1]
    filtered_grid_points, weights = generate_grid_and_weights(blue_row, red_row, obs, spacing=grid_spacing, exclusion_radius=exclusion_rad, weight_function=weight_function)
    start_point, end_point = find_start_end_points(filtered_grid_points, blue_row=blue_row, red_row=red_row)
    return blue_row, red_row, obs, filtered_grid_points, weights, start_point, end_point

def check_point_in_grid(point, grid_points, tolerance=1e-6):
    """
    Check if a point is in the grid by looking for a match within a specified tolerance.

    Parameters:
    - point (tuple or array): The point to check (x, y).
    - grid_points (numpy.ndarray): Array of grid points (Nx2).
    - tolerance (float): The allowable tolerance for matching the point.

    Returns:
    - bool: True if the point is within the grid, False otherwise.
    """
    distances = np.linalg.norm(grid_points - point, axis=1)
    return np.any(distances < tolerance)



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

class NavChannel:
    def __init__(self, grid_spacing=.5, weight_scale=1, exclusion_radius=1.5):
        self.grid_spacing = grid_spacing
        self.weight_scale = weight_scale
        self.exclusion_radius = exclusion_radius

    def scaled_inverse_distance_weight(self, distance):
        return self.weight_scale * inverse_distance_weight(distance)
    
    def getPath(self, red_bouys, blue_bouys, obstacles, start_bouy, end_bouy):
        def poses_to_array(poses):
            array = np.empty((0, 2))
            for pose in poses:
                if not isinstance(pose, Pose) or pose.hasXYCoords():
                    continue
                array.append([pose.x, pose.y])
            return array
        
        red_array = poses_to_array(red_bouys)
        blue_array = poses_to_array(blue_bouys)
        obs_array = poses_to_array(obstacles)
        if isinstance(start_bouy, Pose):
            start_bouy_coords = [start_bouy.x, start_bouy.y]
        if isinstance(end_bouy, Pose):
            end_bouy_coords = [end_bouy.x, start_bouy.y]

        gridpoints, weights = generate_grid_and_weights(blue_row=blue_array, red_row=red_array, obs=obs_array, 
                                               spacing=self.grid_spacing, exclusion_radius=self.exclusion_radius, 
                                               weight_function=self.scaled_inverse_distance_weight)
        graph = create_graph(grid_points=gridpoints, weights=weights, spacing=self.grid_spacing, tolerance=.1)
        if start_bouy_coords is not None:
            endpoint = start_bouy_coords
        elif end_bouy_coords is not None:
            endpoint = end_bouy_coords
        else:
            _ , endpoint - find_start_end_points(grid_points=gridpoints, blue_row=blue_array, red_row=red_array)
        
        startpoint = np.array([0,0])

        start_node = (np.float64(startpoint[0]), np.float64(startpoint[1]))
        end_node = (np.float64(endpoint[0]), np.float64(endpoint[1]))

        try:
            path = astar_path(graph, start_node, end_node, weight='weight', heuristic=heuristic)
        except nx.NetworkXNoPath:
            path = []

        return path





if __name__ == "__main__":
    # Generate example rows for demonstration
    num_points = 2
    start = 0
    end = 10
    noise_level = 5.0
    min_distance = 10.0
    num_random_points = 50

    # Generate grid points and calculate weights
    blue_row, red_row, obs, gridpoints, weights, start_point, end_point = generate_bouys_graph_weights(2,0,20,10,1,10,1,1.5)

    blue_row = blue_row[:1]

    if (start_point.all == None or end_point.all == None):
        print("No start/endpoint")
        plot_grid_points_with_weights_and_rows(gridpoints, blue_row, red_row, obs, start_point, end_point)
        

    # Create and plot the graph
    graph = create_graph(gridpoints, weights, spacing=1)
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

