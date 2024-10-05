#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

class MockGPS:
    def __init__(self):
        rospy.init_node('mock_gps_node', anonymous=True)  # Initialize the ROS node
        self.gps_pub = rospy.Publisher('/gps/gps_fix', NavSatFix, queue_size=10)  # Publisher for GPS fix
        self.rate = rospy.Rate(1)  # Set the rate to 1 Hz

        # Fixed location (latitude, longitude, altitude)
        self.fixed_latitude = 37.7749   # Example latitude (e.g., San Francisco)
        self.fixed_longitude = -122.4194 # Example longitude
        self.fixed_altitude = 0.0        # Altitude (can be set to any value)

    def publish_gps_fix(self):
        """Publish a fixed GPS location."""
        gps_msg = NavSatFix()
        
        gps_msg.header.stamp = rospy.Time.now()  # Set the current time
        gps_msg.header.frame_id = "gps"           # Frame ID for the GPS data
        gps_msg.latitude = self.fixed_latitude     # Set the fixed latitude
        gps_msg.longitude = self.fixed_longitude    # Set the fixed longitude
        gps_msg.altitude = self.fixed_altitude      # Set the fixed altitude

        # # Set status (indicating a valid GPS fix)
        # gps_msg.status.status = NavSatFix.STATUS_FIX
        # gps_msg.status.service = NavSatFix.SERVICE_GPS
        
        # Publish the message
        self.gps_pub.publish(gps_msg)
        rospy.loginfo(f"Published GPS Fix: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.altitude}")

    def run(self):
        """Run the mock GPS node."""
        while not rospy.is_shutdown():
            self.publish_gps_fix()  # Publish the GPS fix
            self.rate.sleep()        # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        mock_gps = MockGPS()  # Create the mock GPS object
        mock_gps.run()        # Run the mock GPS node
    except rospy.ROSInterruptException:
        pass