import rospy
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix  # Corrected import for GPS data
from util.maps import Pose
from math import radians, cos, sin, sqrt, atan2
from typing import List
from gps_bagger.srv import WaypointService, WaypointServiceResponse  # Import the custom service

class WaypointNavigator:
    def __init__(self):
        self.waypoints = []  # List of poses (waypoints)
        self.current_goal_index = 0
        self.current_pose = None  # Store the current GPS location
        self.distance_threshold = 1.0  # 1 meter threshold
        
        # Set up publisher for waypoints and subscriber for current GPS position
        self.waypoint_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
        rospy.Subscriber('/gps/gps_fix', NavSatFix, self.gps_callback)  # Updated to use NavSatFix
        
        # Service to update waypoints
        self.waypoint_service = rospy.Service('/waypoint_service', WaypointService, self.handle_waypoint_service)
        rospy.loginfo("WaypointNavigator service started.")

    def handle_waypoint_service(self, req):
        """Handle the incoming waypoints from the service."""
        try:
            new_waypoints = []
            for wp in req.waypoints:
                # Create Pose objects from the incoming waypoints
                new_waypoints.append(Pose(lat=wp.lat, lon=wp.lon, x=wp.x, y=wp.y, heading=wp.heading, yaw=wp.yaw))

            # Update the waypoints
            self.waypoints = new_waypoints
            self.current_goal_index = 0  # Reset to start with the first waypoint
            rospy.loginfo(f"Received {len(new_waypoints)} new waypoints.")
            return WaypointServiceResponse(success=True)
        
        except Exception as e:
            rospy.logerr(f"Failed to process waypoints: {e}")
            return WaypointServiceResponse(success=False)

    def gps_callback(self, gps_data: NavSatFix):  # Updated to use NavSatFix message type
        """Callback function to get the current GPS position from the /gps/gps_fix topic."""
        self.current_pose = gps_data
        self.check_distance_to_goal()

    def check_distance_to_goal(self):
        """Check the distance between the current GPS position and the goal pose."""
        if self.current_pose is None or self.current_goal_index >= len(self.waypoints):
            return
        
        current_goal = self.waypoints[self.current_goal_index]
        
        if not current_goal.hasGPSCoords():
            rospy.logwarn("Goal pose does not have valid GPS coordinates.")
            return

        distance = self.haversine_distance(self.current_pose.latitude, self.current_pose.longitude,
                                           current_goal.lat, current_goal.lon)

        rospy.loginfo(f"Distance to goal: {distance:.2f} meters")

        if distance <= self.distance_threshold:
            rospy.loginfo(f"Reached waypoint {self.current_goal_index}, sending next waypoint.")
            self.send_next_waypoint()

    def send_next_waypoint(self):
        """Send the next waypoint if available."""
        if self.current_goal_index < len(self.waypoints):
            goal_pose = self.waypoints[self.current_goal_index]
            self.publish_waypoint(goal_pose)
            self.current_goal_index += 1
        else:
            rospy.loginfo("All waypoints have been reached.")

    def publish_waypoint(self, pose: 'Pose'):
        """Publish the current goal pose as a waypoint."""
        waypoint = GlobalPositionTarget()
        
        # Fill in the waypoint message with the GPS coordinates and heading from the Pose object
        waypoint.latitude = pose.lat
        waypoint.longitude = pose.lon
        waypoint.altitude = 0  # Altitude can be customized

        # Set heading/yaw
        waypoint.yaw = pose.heading if pose.heading is not None else 0.0
        
        # Type mask to ignore unused fields (velocity, yaw rate, etc.)
        waypoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        waypoint.type_mask = (GlobalPositionTarget.IGNORE_VX |
                              GlobalPositionTarget.IGNORE_VY |
                              GlobalPositionTarget.IGNORE_VZ |
                              GlobalPositionTarget.IGNORE_AFX |
                              GlobalPositionTarget.IGNORE_AFY |
                              GlobalPositionTarget.IGNORE_AFZ |
                              GlobalPositionTarget.IGNORE_YAW_RATE)
        
        # Publish the waypoint
        self.waypoint_pub.publish(waypoint)
        rospy.loginfo(f"Published waypoint: Latitude: {pose.lat}, Longitude: {pose.lon}, Heading: {pose.heading}")

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate the great-circle distance between two points on the Earth."""
        R = 6371e3  # Radius of Earth in meters
        phi1, phi2 = radians(lat1), radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c  # Distance in meters
