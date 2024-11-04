#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

class GPSRepublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('gps_republisher', anonymous=True)

        # Set up a subscriber to /mavros/global_position/global for GPS data
        self.gps_subscriber = rospy.Subscriber(
            '/mavros/global_position/global',
            NavSatFix,
            self.gps_callback
        )

        # Set up a subscriber to /mavros/global_position/compass_hdg for compass heading
        self.heading_subscriber = rospy.Subscriber(
            '/mavros/global_position/compass_hdg',
            Float64,
            self.heading_callback
        )

        # Set up a publisher to /gps/gps_fix
        self.gps_publisher = rospy.Publisher(
            '/gps/gps_fix',
            NavSatFix,
            queue_size=10
        )

        # Set up a publisher to /heading
        self.heading_publisher = rospy.Publisher(
            '/heading',
            Float64,
            queue_size=10
        )

    def gps_callback(self, msg):
        # Republish the GPS message to /gps/gps_fix
        self.gps_publisher.publish(msg)

    def heading_callback(self, msg):
        # Republish the heading data to /heading
        self.heading_publisher.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        republisher = GPSRepublisher()
        republisher.run()
    except rospy.ROSInterruptException:
        pass
