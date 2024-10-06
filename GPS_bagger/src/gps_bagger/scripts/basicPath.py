#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from gps_bagger.srv import WaypointService, WaypointServiceRequest
from util.maps import Pose
from util.path_function import basicPaths


    

class BasicPath:
    def __init__(self):
        # Initialize the node
        rospy.init_node('basicPath', anonymous=True)
        try:
            gps_data = rospy.wait_for_message('/gps/gps_fix', NavSatFix, timeout=10)
            hdt = rospy.wait_for_message('/heading', Float64, timeout=10)
            self.current_pose = Pose(lat=gps_data.latitude, lon=gps_data.longitude, heading=float(hdt.data), x=0, y=0)
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive GPS data: %s", str(e))
            pass
        # Load initial parameters from the parameter server
        self.update_parameters()
        if self.path_name not in basicPaths.pose_functions.keys():
            print("path not found")
            pass
        poses = basicPaths.pose_functions[self.path_name](
            interval=self.interval,distance=self.distance, radius=self.radius,
            amplitude=self.amplitude, direction=self.direction, current_pose=self.current_pose)
        if not self.waypoint_service_client(poses):
            rospy.logerr("Failed to call waypoint service")
        pass
        

    def update_parameters(self):
        """Load parameters from the ROS parameter server."""
        self.path_name = rospy.get_param('~path_name', "straight line")  # ~ means private parameter (namespaced to the node)
        self.interval = rospy.get_param('~interval', 2)
        self.distance = rospy.get_param('~distance', 20)
        self.amplitude = rospy.get_param('~amplitude', 20)
        self.direction = rospy.get_param('~direction', 'clockwise')  # Default values provided
        self.radius = rospy.get_param('~radius', 10)

    def waypoint_service_client(self, poses):
        """Call the WaypointService with the given waypoints and headings."""
        rospy.wait_for_service('/waypoint_service')  # Wait until the service is available
        try:
            waypoint_service = rospy.ServiceProxy('/waypoint_service', WaypointService)
            request = WaypointServiceRequest()
            waypoints = []
            headings = []
            for pose in poses:
                waypoint = NavSatFix()
                waypoint.latitude = pose.lat
                waypoint.longitude = pose.lon
                waypoints.append(waypoint)
                headings.append(pose.heading)
            request.waypoints = waypoints  # Assign waypoints to the request
            request.headings = headings     # Assign headings to the request
            response = waypoint_service(request)  # Call the service
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

if __name__ == '__main__':
    try:
        node = BasicPath()
    except rospy.ROSInterruptException:
        pass

