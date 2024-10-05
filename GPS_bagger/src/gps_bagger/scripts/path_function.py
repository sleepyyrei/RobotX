from util.maps import Pose
import numpy as np
import matplotlib.pyplot as plt

def angle_with_x_axis(vector):
    """Calculate the angle in degrees between a vector and the x-axis."""
    x, y = vector
    # Calculate the angle in radians
    angle_rad = np.arctan2(y, x)
    
    # Convert to degrees
    angle_deg = np.degrees(angle_rad)
    
    # Ensure the angle is in the range [0, 360)
    if angle_deg < 0:
        angle_deg += 360
        
    return angle_deg

def generate_sine_wave(start, end, num_points, period, amplitude):
    """Generate x-values and corresponding sine y-values with specified period and amplitude."""
    x_values = np.linspace(start, end, num_points)
    # Scale the sine function to set the desired period and amplitude
    y_values = amplitude * np.sin((2 * np.pi / period) * x_values)  
    return x_values, y_values

def generate_circle_segment(start_point, radius, angle, direction, num_points=100):
    """Generate points on a segment of a circle given a start point, radius, angle, and direction."""
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle)

    # Determine the center of the circle based on the start point and angle
    if direction == 'clockwise':
        # Calculate center for clockwise direction
        center_x = start_point[0]
        center_y = start_point[1] + radius 
    else:  # anticlockwise
        # Calculate center for anticlockwise direction
        center_x = start_point[0]
        center_y = start_point[1] - radius

    # Generate angles for the segment
    angles = np.linspace(np.pi/2, np.pi/2 - angle_rad, num_points)
    x = center_x + radius * np.cos(angles)  # Adjust for center x-coordinate
    y = center_y + radius * np.sin(angles)  # Adjust for center y-coordinate
    return x, y

def sample_points_at_interval(x_values, y_values, interval):
    """Sample points approximately at the specified interval."""
    sampled_x = []
    sampled_y = []
    
    last_x = None
    last_y = None
    for x, y in zip(x_values, y_values):
        if last_x is None or (((y - last_y)**2 + (x - last_x)**2)**.5 >= interval):
            sampled_x.append(x)
            sampled_y.append(y)
            last_x = x
            last_y = y
            
    # Add the last point (end point)
    if (sampled_x[-1] != x_values[-1] or sampled_y[-1] != y_values[-1]):
        sampled_x.append(x_values[-1])
        sampled_y.append(y_values[-1])
    return sampled_x, sampled_y

def generate_yaw(x_values, y_values):
    length = len(x_values)
    i = 0
    yaws = []
    for i in range(len(x_values) - 1):
        dx = x_values[i + 1] - x_values[i]
        dy = y_values[i + 1] - y_values[i]
        yaw = -angle_with_x_axis((dx, dy))
        yaws.append(yaw)
    yaws.append(yaw)
    print(yaws)
    return yaws

def generate_poses(x_values, y_values, yaws, origin):
    if not isinstance(origin, Pose) or not origin.isValidOrigin():
        return None
    poses = []
    for x, y, yaw in zip(x_values, y_values, yaws):
        new_pose = Pose(x=x, y=y, yaw=yaw, heading=None, origin=origin)
        print(f"New Pose -  {new_pose.lat}, {new_pose.lon}, Heading: {new_pose.heading}")
        poses.append(new_pose)
    return poses

def generateLine(current_pose, interval, distance):
    try:
        interval = float(interval)
        distance = float(distance)
    except:
        pass
    if (not isinstance(current_pose, Pose) or not current_pose.isValidOrigin() 
        or not isinstance(interval, float) or not isinstance(distance, float)):
        return None
    line_distance = 0
    poses = []
    while line_distance < distance:
        new_pose = Pose(x=line_distance, y=0, yaw=0, origin=current_pose, heading=None)
        poses.append(new_pose)
        # print(f"New Pose -  {new_pose.lat}, {new_pose.lon}, Heading: {new_pose.heading}")
        line_distance += interval
    return poses

def generateSine(current_pose, interval, amplitude, distance):
    x_values, y_values = generate_sine_wave(0, distance, 1000, distance, amplitude)
    x_values, y_values = sample_points_at_interval(x_values, y_values, interval)
    yaws = generate_yaw(x_values, y_values)
    generate_poses(x_values, y_values, yaws, current_pose)



if __name__ == "__main__":
   # Parameters
    start_point = (1, 2)    # Start point (x, y)
    radius = 5               # Circle radius
    angle = 180               # Angle in degrees
    direction = 'anticlockwise'  # Direction ('clockwise' or 'anticlockwise')
    num_points = 100         # Number of points to generate

    x_line = [-9,-7,-5,-3,-1,1]
    y_line = [2 for _ in range(6)]
    # Generate circle segment points
    x_circle, y_circle = generate_circle_segment(start_point, radius, angle, direction, num_points)
    x_circle, y_circle = sample_points_at_interval(x_circle, y_circle, 2)
    y_line.extend(y_circle)
    x_line.extend(x_circle)
    print(x_line, y_line)
    # Plotting the circle segment
    plt.figure(figsize=(6, 6))
    plt.plot(x_line, y_line, label='Circle Segment', color='blue')
    plt.scatter(*start_point, color='red', label='Start Point', zorder=5)  # Mark the start point
    plt.gca().set_aspect('equal', adjustable='box')
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
    # Set equal scaling for both axes
    plt.axis('equal')  # Ensure equal scaling
    plt.title(f'Circle Segment from Start Point {start_point} (Angle: {angle}°, Direction: {direction})')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid()
    plt.legend()
    plt.show()
