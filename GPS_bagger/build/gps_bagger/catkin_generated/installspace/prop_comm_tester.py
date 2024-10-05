#!/usr/bin/env python3

import rospy
from gps_bagger.srv import WaypointService, WaypointServiceRequest
from sensor_msgs.msg import NavSatFix

def waypoint_service_client(waypoints, headings):
    """Call the WaypointService with the given waypoints and headings."""
    rospy.wait_for_service('/waypoint_service')  # Wait until the service is available
    try:
        waypoint_service = rospy.ServiceProxy('/waypoint_service', WaypointService)
        request = WaypointServiceRequest()
        request.waypoints = waypoints  # Assign waypoints to the request
        request.headings = headings     # Assign headings to the request
        response = waypoint_service(request)  # Call the service
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def main():
    rospy.init_node('waypoint_service_test_node', anonymous=True)  # Initialize the test node
    rospy.loginfo("Waypoint Service Test Node is running...")

    # Create a list of waypoints (sensor_msgs/NavSatFix messages)
    waypoints = []
    headings = []  # This will hold heading values

    # Example waypoints to test (make sure these are valid for your application)
    # Create waypoints using NavSatFix messages
    for lat, lon, heading in [(37.7749, -122.4194, 0.0), (34.0522, -118.2437, 90.0)]:
        waypoint = NavSatFix()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "map"  # Set appropriate frame_id
        waypoint.latitude = lat
        waypoint.longitude = lon
        waypoint.altitude = 0.0  # Set altitude as needed

        waypoints.append(waypoint)
        headings.append(heading)  # Append the heading for each waypoint

    # Call the service and print the response
    success = waypoint_service_client(waypoints, headings)
    if success:
        rospy.loginfo("Waypoints successfully sent to the waypoint service.")
    else:
        rospy.logwarn("Failed to send waypoints to the waypoint service.")

if __name__ == '__main__':
    main()  # Run the main function when the script is executed
