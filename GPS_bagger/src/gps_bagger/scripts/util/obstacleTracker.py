# from util.maps import Obstacle
from maps import Obstacle, Pose
import math
import numpy as np
import copy

class KalmanFilterGPS:
    def __init__(self, initial_state, initial_uncertainty, process_variance):
        # Initialize geodetic state (latitude, longitude)
        self.state = np.array(initial_state)  # [latitude, longitude]
        self.uncertainty = np.array(initial_uncertainty)  # Initial uncertainty in lat/lon
        self.process_variance = process_variance

    def predict(self):
        # Prediction step (assuming stationary or minimal drift for simplicity)
        self.uncertainty += self.process_variance
        return self.state, self.uncertainty

    def update(self, measurement_lat, measurement_lon, measurement_variance_xy, boat_lat):
        # Convert relative XY measurement to GPS coordinates
        measurement_gps = np.array([measurement_lat, measurement_lon])

        # Convert measurement variance from XY (meters) to GPS (degrees) based on latitude
        measurement_variance_gps = KalmanFilterGPS.convert_variance_to_gps(measurement_variance_xy, boat_lat)

        # Kalman Gain computation using the converted GPS measurement variance
        S = self.uncertainty + measurement_variance_gps
        K = self.uncertainty @ np.linalg.inv(S)  # Change to matrix multiplication

        # Update the geodetic state estimate
        self.state += K @ (measurement_gps - self.state)  # Update state using matrix multiplication
        self.uncertainty = (np.eye(2) - K) @ self.uncertainty  # Update uncertainty

        return self.state, self.uncertainty

    @staticmethod
    def convert_variance_to_gps(variance_xy, latitude):
        # Separate x and y variances for latitude and longitude in GPS
        variance_x, variance_y = variance_xy  # Variances for x and y in meters squared
        variance_lat = variance_x / (111_320**2)  # Convert x variance to latitude in degrees squared
        variance_lon = variance_y / (111_320 * np.cos(np.radians(latitude)))**2  # Convert y variance to longitude
        return np.diag([variance_lat, variance_lon])


class ObstacleKalman(KalmanFilterGPS):
    process_variance = 0.0001
    def __init__(self, initial_pose):
        # Convert pose to 0 heading to get ENU coordinates
        newOrigin = initial_pose.origin
        newOrigin.heading = 0
        enu_pose = initial_pose.changeOrigin(newOrigin)
        measurement_uncertainty = [(enu_pose.x * 0.01)**2, (enu_pose.y * 0.01)**2]

        super().__init__(
            [initial_pose.lat, initial_pose.lon],
            KalmanFilterGPS.convert_variance_to_gps(measurement_uncertainty, newOrigin.lat),
            np.diag([ObstacleKalman.process_variance, ObstacleKalman.process_variance])
        )
        self.detection_count = 0
        self.non_detection_count = 0
        self.pose = Obstacle(initial_pose.lat, initial_pose.lon, origin=initial_pose.origin, 
                             heading=0, object_type=initial_pose.object_type, 
                             colour=initial_pose.colour)
    
    def predict(self):
            # Prediction step (assuming stationary or minimal drift for simplicity)
            self.uncertainty += ObstacleKalman.process_variance
            return self.state, self.uncertainty

    def update(self, new_pose):
        self.predict()

        # Convert pose to 0 heading to get ENU coordinates
        newOrigin = new_pose.origin
        newOrigin.heading = 0
        enu_pose = new_pose.changeOrigin(newOrigin)
        measurement_uncertainty = [(enu_pose.x * 0.01)**2, (enu_pose.y * 0.01)**2]
        
        # Call update method from the parent class
        updated_state, updated_uncertainty = super().update(
            new_pose.lat, new_pose.lon, measurement_uncertainty, newOrigin.lat
        )
        
        print(updated_state)

        self.detection_count += 1
        self.pose = Obstacle(lat=float(updated_state[0]), lon=float(updated_state[1]), 
                             origin=new_pose.origin, heading=0, object_type=new_pose.object_type, 
                             colour=new_pose.colour)
        print(f"{self.pose.x} ,{self.pose.y}" )
        return updated_state, updated_uncertainty
    
    def __eq__(self, value: object) -> bool:
        if not isinstance(value, ObstacleKalman):
            return False
        return self.pose == value.pose

class ObstacleTracker:
    # Dictionary to define tolerance for each obstacle type (in meters)
    obstacle_tolerance_dict = {
        "bouy": 10,
        "dock_panel": 6
    }

    def __init__(self, current_pose, tolerance=3, warning_tolerence=0.5, memory_range=30, non_detection_tolerance=5):
        # Initialize with current boat pose, default tolerances, and memory range
        if current_pose is not None and isinstance(current_pose, Pose) and current_pose.isValidOrigin():
            self.current_pose = current_pose
            self.heading = current_pose.heading
        self.tolerance = tolerance  # Default tolerance for unlisted obstacle types
        self.warning_tolerance = warning_tolerence  # Tolerance for warning state
        self.non_detection_tolerance = non_detection_tolerance
        self.obstacles = []  # List to store currently tracked obstacles
        self.new_obstacle_flag = False  # Flag for detecting new obstacles
        self.obstacle_moved_flag = False  # Flag for obstacle movement

    def dynamic_reconfig_callback(self, config, level):
        # Callback for dynamically reconfiguring parameters
        self.tolerance = config['obstacle_tolerance']
        ObstacleKalman.process_variance = config['process_variance']
        return config

    def gps_callback(self, msg):
        # Update current boat pose based on GPS data
        self.current_pose = Pose(lat=msg.latitude, lon=msg.longitude, heading=self.heading, yaw=0, x=0, y=0)
        for obstacle in self.obstacles:
            obstacle.pose.changeOrigin(self.current_pose, True)

    def heading_callback(self, msg):
        # Update boat heading based on new heading data
        self.heading = float(msg.data)
        self.current_pose.heading = self.heading
        for obstacle in self.obstacles:
            obstacle.pose.changeOrigin(self.current_pose, True)


    def obstacle_tracking(self, obstacles):
        # Clone new and old obstacle lists to prevent modification during iteration
        new_obstacles_clone = copy.deepcopy(obstacles)
        old_obstacle_clone = copy.deepcopy(self.obstacles)

        # Loop through each new obstacle to find matches in old obstacles
        for new_obstacle in new_obstacles_clone:
            closest_distance = 1000  # Initialize to a high value to find minimum distance
            closest_old_obstacle = None  # Variable to hold closest matched old obstacle

            # Set tolerance for specific obstacle types, defaulting to `self.tolerance`
            if new_obstacle.object_type in ObstacleTracker.obstacle_tolerance_dict:
                object_tolerance = ObstacleTracker.obstacle_tolerance_dict[new_obstacle.object_type]
            else:
                object_tolerance = self.tolerance

            # Compare the new obstacle with each old obstacle
            for old_obstacle in old_obstacle_clone:
                # Check if the obstacle types match
                if old_obstacle.pose.sameObjectType(new_obstacle):
                    # Calculate distance between the new and old obstacle positions
                    distance = ObstacleTracker.haversine(
                        new_obstacle.lat, new_obstacle.lon, 
                        old_obstacle.pose.lat, old_obstacle.pose.lon
                    )
                    # Update the closest obstacle if within tolerance and distance is minimal
                    if distance < object_tolerance and distance < closest_distance:
                        closest_distance = distance
                        closest_old_obstacle = old_obstacle

            # If a matching old obstacle is found within tolerance, update it with new data
            if closest_old_obstacle is not None:
                for actual_obstacle in self.obstacles:
                    if actual_obstacle == closest_old_obstacle:
                        old_obstacle_clone.remove(closest_old_obstacle)
                        closest_old_obstacle = actual_obstacle
                        new_obstacles_clone.remove(new_obstacle)  # Remove from new list once matched
                        closest_old_obstacle.update(new_obstacle)  # Update closest old obstacle
                        
                
            
        for old_obstacle in old_obstacle_clone:
            for actual_obstacle in self.obstacles:
                    if actual_obstacle == old_obstacle:
                        old_obstacle = actual_obstacle
                        old_obstacle.non_detection_count += 1
                        if old_obstacle.non_detection_count > self.non_detection_tolerance:
                            self.obstacles.remove(old_obstacle)
        
        for new_obstacle in new_obstacles_clone:
            if new_obstacle.hasXYCoords() and new_obstacle.hasGPSCoords():
                
                self.obstacles.append(ObstacleKalman(new_obstacle))
                self.new_obstacle_flag = True
    

        
                    


    
    def haversine(lat1, lon1, lat2, lon2):
        """
        Calculate the distance between two GPS points (latitude and longitude) using the Haversine formula.

        Parameters:
        lat1, lon1: Latitude and Longitude of point 1 (in decimal degrees)
        lat2, lon2: Latitude and Longitude of point 2 (in decimal degrees)

        Returns:
        Distance between the two points in kilometers.
        """
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Difference in coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in kilometers
        distance = R * c

        return distance
    
if __name__ == "__main__":
   # Assume the above classes have been imported correctly, especially the Obstacle and Pose classes

    # Initialize the boat's current position
    current_pose = Pose(lat=34.0, lon=-117.0, heading=0, yaw=0, x=0, y=0)
    tracker = ObstacleTracker(current_pose=current_pose)

    # Initial set of detected obstacles with known positions
    initial_obstacles = [
        Obstacle(x=10, y=5, origin=current_pose, heading=0, object_type="bouy", colour="red"),
        Obstacle(lat=34.0002, lon=-117.0002, origin=current_pose, heading=0, object_type="dock_panel", colour="blue"),
        Obstacle(lat=34.0003, lon=-117.0003, origin=current_pose, heading=0, object_type="bouy", colour="green"),
    ]

    # Feed initial detections into the tracker
    tracker.obstacle_tracking(initial_obstacles)

    print("\n--- Initial Obstacle State ---")
    for obstacle in tracker.obstacles:
        print(f"Obstacle: Type = {obstacle.pose.object_type}, Latitude = {obstacle.pose.lat}, Longitude = {obstacle.pose.lon}, Colour = {obstacle.pose.colour}")

    # Simulate a sequence of new readings for obstacles (some existing, some new)
    new_readings_sequence = [
        [
            Obstacle(x=10.5, y=5.5, origin=current_pose, heading=0, object_type="bouy", colour="red"),
            # Obstacle(lat=34.00025, lon=-117.00025, origin=current_pose, heading=0, object_type="dock_panel", colour="blue"),
            # Obstacle(lat=34.0004, lon=-117.00035, origin=current_pose, heading=0, object_type="bouy", colour="yellow"),  # New obstacle
        ],
        [
            Obstacle(x=11, y=6, origin=current_pose, heading=0, object_type="bouy", colour="red"),
            # Obstacle(lat=34.00028, lon=-117.00028, origin=current_pose, heading=0, object_type="dock_panel", colour="blue"),
            # Obstacle(lat=34.0005, lon=-117.00045, origin=current_pose, heading=0, object_type="bouy", colour="yellow"),
        ],
        [
            Obstacle(x=15, y=0, origin=current_pose, heading=0, object_type="bouy", colour="red"),
            # Obstacle(lat=34.0003, lon=-117.0003, origin=current_pose, heading=0, object_type="dock_panel", colour="blue"),
            # Obstacle(lat=34.00055, lon=-117.00055, origin=current_pose, heading=0, object_type="bouy", colour="yellow"),
        ],
    ]

    print(new_readings_sequence[0][0].lat)

    # Feed each set of readings into the tracker sequentially
    for i, readings in enumerate(new_readings_sequence):
        print(f"\n--- Update {i + 1} ---")
        tracker.obstacle_tracking(readings)

        # Print updated obstacle states
        for obstacle in tracker.obstacles:
            print(f"Obstacle: {obstacle.pose.colour} {obstacle.pose.object_type}, GPS: {obstacle.pose.lat}, {obstacle.pose.lon}, XY: {obstacle.pose.x}, {obstacle.pose.y} , Non-Detection Count = {obstacle.non_detection_count}")

        # Check for new and moved obstacles flags
        print(f"New Obstacle Flag: {tracker.new_obstacle_flag}")
        print(f"Obstacle Moved Flag: {tracker.obstacle_moved_flag}")

        # Reset flags after each update for clarity in subsequent iterations
        tracker.new_obstacle_flag = False
        tracker.obstacle_moved_flag = False

    class GPSmessage:
        def __init__(self, lat, lon) -> None:
            self.latitude = lat
            self.longitude = lon
    
    msg = GPSmessage(lat=34.000008, lon=-117.000001)
    tracker.gps_callback(msg)
    print(tracker.current_pose.lat)
    for obstacle in tracker.obstacles:
            print(f"Obstacle: {obstacle.pose.colour} {obstacle.pose.object_type}, GPS: {obstacle.pose.lat}, {obstacle.pose.lon}, XY: {obstacle.pose.x}, {obstacle.pose.y} , Non-Detection Count = {obstacle.non_detection_count}")