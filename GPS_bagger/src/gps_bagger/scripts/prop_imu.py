#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class IMUHeadingCalculator:
    def __init__(self):
        rospy.init_node('imu_heading_calculator', anonymous=True)
        
        # Publisher for heading
        self.heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)
        
        # Subscriber to IMU data
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        
        self.rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

    def imu_callback(self, imu_data):
        """Callback function to process incoming IMU data."""
        # Extract magnetometer data from IMU message
        mag_x = imu_data.magnetic_field.x
        mag_y = imu_data.magnetic_field.y
        
        # Calculate heading from magnetometer readings
        heading = self.calculate_heading(mag_x, mag_y)
        
        # Publish the heading
        self.publish_heading(heading)

    def calculate_heading(self, mag_x, mag_y):
        """Calculate heading from magnetometer readings."""
        heading = math.atan2(mag_y, mag_x)  # Heading in radians
        heading = math.degrees(heading)      # Convert to degrees
        heading = (heading + 360) % 360      # Normalize to [0, 360)
        return heading

    def publish_heading(self, heading):
        """Publish heading as a Float64 message."""
        heading_msg = Float64()
        heading_msg.data = heading
        self.heading_pub.publish(heading_msg)
        rospy.loginfo(f"Published Heading: {heading:.2f} degrees")

    def run(self):
        """Run the ROS node."""
        while not rospy.is_shutdown():
            self.rate.sleep()  # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        heading_calculator = IMUHeadingCalculator()  # Create the heading calculator object
        heading_calculator.run()                      # Run the ROS node
    except rospy.ROSInterruptException:
        pass
