#!/usr/bin/env python

import rospy
from gps_bagger.srv import WaypointService, WaypointServiceRequest

def waypoint_service_client(waypoints):
    """Call the WaypointService with the given waypoints."""
    rospy.wait_for_service('/waypoint_service')  # Wait until the service is available
    try:
        waypoint_service = rospy.ServiceProxy('/waypoint_service', WaypointService)
        request = WaypointServiceRequest()
        request.waypoints = waypoints  # Assign waypoints to the request
        response = waypoint_service(request)  # Call the service
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def main():
    rospy.init_node('waypoint_service_test_node', anonymous=True)  # Initialize the test node
    rospy.loginfo("Waypoint Service Test Node is running...")

    # Example waypoints to test (make sure these are valid for your application)
    waypoints = [
        # Create waypoints with (lat, lon, heading)
        # Assuming your waypoint structure requires lat, lon, and heading
        {'lat': 37.7749, 'lon': -122.4194, 'heading': 0.0},  # Example waypoint 1
        {'lat': 34.0522, 'lon': -118.2437, 'heading': 90.0},  # Example waypoint 2
    ]

    # Call the service and print the response
    success = waypoint_service_client(waypoints)
    if success:
        rospy.loginfo("Waypoints successfully sent to the waypoint service.")
    else:
        rospy.logwarn("Failed to send waypoints to the waypoint service.")

if __name__ == '__main__':
    main()  # Run the main function when the script is executed