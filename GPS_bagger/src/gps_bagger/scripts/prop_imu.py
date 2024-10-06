#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import tf

class IMUHeadingCalculator:
    def __init__(self):
        rospy.init_node('imu_heading_calculator', anonymous=True)
        
        # Publisher for heading
        self.heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)
        
        # Subscriber to IMU data
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        
        self.rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

    def imu_callback(self, data):
        """Callback function to process incoming IMU data."""
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
    
        # Convert quaternion to roll, pitch, and yaw
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        # Publish the heading
        self.publish_heading(yaw)

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
