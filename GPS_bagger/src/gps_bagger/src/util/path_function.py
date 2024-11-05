from util.maps import Pose
import numpy as np
import matplotlib.pyplot as plt

class basicPaths:

    @staticmethod
    def angle_with_x_axis(vector):
        """Calculate the angle in degrees between a vector and the x-axis."""
        x, y = vector
        angle_rad = np.arctan2(y, x)  # Calculate the angle in radians
        angle_deg = np.degrees(angle_rad)  # Convert to degrees
        
        # Ensure the angle is in the range [0, 360)
        return angle_deg if angle_deg >= 0 else angle_deg + 360

    @staticmethod
    def generate_line(start, distance, interval):
        """Generate a line from start with given distance and interval."""
        # Generate x array
        x = np.arange(start[0], start[0] + distance, interval).tolist()
        x.append(start[0] + distance)  # Ensure the endpoint is included
        
        # Make y an array filled with start[1], same length as x
        y = [start[1]] * len(x)
        return x, y


    @staticmethod
    def generate_sine_wave(start, end, num_points, period, amplitude):
        """Generate x-values and corresponding sine y-values with specified period and amplitude."""
        x_values = np.linspace(start, end, num_points)
        y_values = (amplitude * np.sin((2 * np.pi / period) * x_values)).tolist()
        return x_values, y_values

    @staticmethod
    def generate_circle_segment(start_point, radius, angle, direction, num_points=100):
        """Generate points on a segment of a circle given a start point, radius, angle, and direction."""
        angle_rad = np.radians(angle)  # Convert angle from degrees to radians
        center_x, center_y = start_point
        
        # Calculate center based on the start point and angle
        if direction == 'clockwise':
            center_y += radius
            angles = np.linspace(-np.pi/2, -np.pi/2 + angle_rad, num_points)
        else:  # anticlockwise
            center_y -= radius
            angles = np.linspace(np.pi/2, np.pi/2 - angle_rad, num_points)

        x = center_x + radius * np.cos(angles)  # Adjust for center x-coordinate
        y = center_y + radius * np.sin(angles)  # Adjust for center y-coordinate
        return x.tolist(), y.tolist()

    @staticmethod
    def sample_points_at_interval(x_values, y_values, interval):
        """Sample points approximately at the specified interval."""
        sampled_x = []
        sampled_y = []
        
        last_x = None
        last_y = None
        for x, y in zip(x_values, y_values):
            if last_x is None or (((y - last_y) ** 2 + (x - last_x) ** 2) ** 0.5 >= interval):
                sampled_x.append(x)
                sampled_y.append(y)
                last_x = x
                last_y = y

        # Add the last point (end point)
        if sampled_x and (sampled_x[-1] != x_values[-1] or sampled_y[-1] != y_values[-1]):
            sampled_x.append(x_values[-1])
            sampled_y.append(y_values[-1])
            
        return sampled_x, sampled_y

    def generate_search_pattern(x, y):
        x_values = [x, 2*x]
        y_values = [y, -y]
        yaws = [-np.arctan(y/x), np.arctan(y/x)] 
        return x_values, y_values, yaws


    @staticmethod
    def generate_yaw(x_values, y_values):
        """Generate yaw angles based on changes in x and y values."""
        yaws = []
        for i in range(len(x_values) - 1):
            dx = x_values[i + 1] - x_values[i]
            dy = y_values[i + 1] - y_values[i]
            yaw = float(-basicPaths.angle_with_x_axis((dx, dy)))  # Calculate yaw angle
            yaws.append(yaw)
        
        if yaws:  # Append last yaw value
            yaws.append(yaws[-1])
        
        return yaws

    @staticmethod
    def generate_poses(x_values, y_values, yaws, origin):
        """Generate Pose objects from x, y coordinates and yaw values."""
        if not isinstance(origin, Pose) or not origin.isValidOrigin():
            return None
            
        poses = []
        for x, y, yaw in zip(x_values, y_values, yaws):
            new_pose = Pose(x=x, y=y, yaw=yaw, heading=None, origin=origin)
            poses.append(new_pose)
        return poses

    @staticmethod
    def generateLinePose(current_pose, interval, distance, **kwargs):
        """Generate poses along a straight line."""
        try:
            interval = float(interval)
            distance = float(distance)
        except ValueError:
            return None
            
        if not isinstance(current_pose, Pose) or not current_pose.isValidOrigin() or not isinstance(interval, float) or not isinstance(distance, float):
            return None
            
        line_distance = 0
        poses = []
        while line_distance <= distance:
            new_pose = Pose(x=line_distance, y=0, yaw=0, origin=current_pose, heading=None)
            poses.append(new_pose)
            line_distance += interval
            
        return poses

    @staticmethod
    def generateSinePose(current_pose, interval, amplitude, distance, **kwargs):
        """Generate poses along a sine wave."""
        x_values, y_values = basicPaths.generate_sine_wave(0, distance, 1000, distance, amplitude)
        x_values, y_values = basicPaths.sample_points_at_interval(x_values, y_values, interval)
        yaws = basicPaths.generate_yaw(x_values, y_values)
        return basicPaths.generate_poses(x_values, y_values, yaws, current_pose)

    @staticmethod
    def generateCirclePose(current_pose, interval, distance, radius, direction, **kwargs):
        """Generate poses along a circular path."""
        x_line, y_line = basicPaths.generate_line((0, 0), distance, interval)
        x_circle, y_circle = basicPaths.generate_circle_segment((x_line[-1], y_line[-1]), radius, 360, direction, 100)
        x_circle, y_circle = basicPaths.sample_points_at_interval(x_circle, y_circle, interval)
        x_line.extend(x_circle)
        y_line.extend(y_circle)
        poses = basicPaths.generate_poses(x_line, y_line, basicPaths.generate_yaw(x_line, y_line), current_pose)
        return poses

    @staticmethod
    def generateUTurnPose(current_pose, interval, distance, radius, direction, **kwargs):
        """Generate poses for a U-turn path."""
        x_line, y_line = basicPaths.generate_line((0, 0), distance, interval)
        x_circle, y_circle = basicPaths.generate_circle_segment((x_line[-1], y_line[-1]), radius, 180, direction, 100)
        x_circle, y_circle = basicPaths.sample_points_at_interval(x_circle, y_circle, interval)
        x_back, y_back = basicPaths.generate_line((x_circle[-1], y_circle[-1]), distance, interval)
        
        # Combine all segments
        x = x_line + x_circle + x_back
        y = y_line + y_circle + y_back
        poses = basicPaths.generate_poses(x, y, basicPaths.generate_yaw(x, y), current_pose)
        return poses
    def generateStationaryPose(current_pose, **kwargs):
        return basicPaths.generateLinePose(current_pose=current_pose, interval=1, distance=0)
    
    @staticmethod
    def generate_search_poses(current_pose, x, y):
        x_values, y_values, yaw  = basicPaths.generate_search_pattern(x, y)
        return basicPaths.generate_poses(x_values=x_values, y_values=y_values,yaws=yaw)

    @staticmethod
    def entrance_exit_gate(gates, black bouy):
        if gates is not None:
            if len(gates > 1):

                def find_midpoint(x1,y1,x2,y2,d):
                
                    # Step 1: Calculate the midpoint
                    mid_x = (x1 + x2) / 2
                    mid_y = (y1 + y2) / 2

                    # Step 2: Calculate the slope of the line between the two points
                    if x2 != x1:  # Avoid division by zero for a vertical line
                        m = (y2 - y1) / (x2 - x1)
                        # Step 3: Calculate the slope of the perpendicular line
                        m_perp = -1 / m
                    else:
                        m_perp = 0  # For a vertical line, the perpendicular is horizontal

                    # Step 4: Calculate the coordinates of the points at distance d from the midpoint
                    # Use both positive and negative directions along the perpendicular line
                    dx = d / np.sqrt(1 + m_perp**2)
                    dy = m_perp * dx

                    # Calculate the two points
                    point1 = (mid_x + dx, mid_y + dy)
                    point2 = (mid_x - dx, mid_y - dy)
                    return [mid_x, mid_y], point2
                
        
                pass


    # Store pose generation functions
basicPaths.pose_functions = {
    "straight line": basicPaths.generateLinePose,
    "sine wave": basicPaths.generateSinePose,
    "circle": basicPaths.generateCirclePose,
    "U-turn": basicPaths.generateUTurnPose,
    "stationary" : basicPaths.generateStationaryPose
}



if __name__ == "__main__":
   # Parameters
    start_point = (1, 2)    # Start point (x, y)
    radius = 5               # Circle radius
    angle = 180               # Angle in degrees
    direction = 'anticlockwise'  # Direction ('clockwise' or 'anticlockwise')
    num_points = 100         # Number of points to generate
    origin = Pose(lat=37.7749, lon=-122.4194, x=0.0, y=0.0, origin=None, heading=0)
    poses = basicPaths.generateUTurnPose(origin, 2, 10, 5, 'clockwise', amplitude=2)
    # print(x_line, y_line)
    # # Plotting the circle segment
    # plt.figure(figsize=(6, 6))
    # plt.plot(x_line, y_line, label='Circle Segment', color='blue')
    # plt.scatter(*start_point, color='red', label='Start Point', zorder=5)  # Mark the start point
    # plt.gca().set_aspect('equal', adjustable='box')
    # plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    # plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
    # # Set equal scaling for both axes
    # plt.axis('equal')  # Ensure equal scaling
    # plt.title(f'Circle Segment from Start Point {start_point} (Angle: {angle}°, Direction: {direction})')
    # plt.xlabel('X-axis')
    # plt.ylabel('Y-axis')
    # plt.grid()
    # plt.legend()
    # plt.show()
