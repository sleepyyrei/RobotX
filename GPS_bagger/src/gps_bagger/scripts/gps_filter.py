#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import serial
from util.parser import parse_gpgga, parse_hdt
from util.common_utilities import gps_to_enu
import math
import time

# Threshold for acceleration (in meters per second squared)
ACCELERATION_THRESHOLD = 5.0

# Initialize variables for previous GPS data and time
last_lat = None
last_lon = None
last_alt = None
last_time = None
last_velocity = 0.0

def calculate_velocity(lat, lon, alt, timestamp):
    global last_lat, last_lon, last_alt, last_time, last_velocity

    if last_lat is None or last_lon is None or last_time is None:
        # First GPS reading, no velocity or acceleration calculation
        last_lat = lat
        last_lon = lon
        last_alt = alt
        last_time = timestamp
        return 0.0

    # Calculate time difference in seconds
    time_diff = timestamp - last_time
    if time_diff == 0:
        return last_velocity  # Avoid division by zero

    # Convert the two GPS positions to ENU coordinates
    x1, y1, _ = gps_to_enu(last_lat, last_lon, last_alt, baseLat, baseLon)
    x2, y2, _ = gps_to_enu(lat, lon, alt, baseLat, baseLon)

    # Calculate the distance traveled (Euclidean distance)
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Calculate velocity (distance over time)
    velocity = distance / time_diff

    last_lat = lat
    last_lon = lon
    last_alt = alt
    last_time = timestamp
    last_velocity = velocity

    return velocity

def calculate_acceleration(current_velocity, previous_velocity, time_diff):
    return (current_velocity - previous_velocity) / time_diff if time_diff > 0 else 0

def read_gps_data():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.loginfo("Connected to GPS")

    last_velocity = 0.0
    last_time = time.time()

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            sentence = ser.readline().decode('ascii', errors='replace').strip()
            if sentence.startswith('$GPGGA'):
                lat, lon, alt = parse_gpgga(sentence)

                if lat is not None and lon is not None:
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = rospy.Time.now()
                    gps_msg.header.frame_id = "gps"
                    gps_msg.latitude = lat
                    gps_msg.longitude = lon
                    gps_msg.altitude = alt

                    # Get the current time and calculate velocity
                    current_time = time.time()
                    velocity = calculate_velocity(lat, lon, alt, current_time)

                    # Calculate acceleration
                    acceleration = calculate_acceleration(velocity, last_velocity, current_time - last_time)

                    # Filter out GPS data if acceleration exceeds the threshold
                    if abs(acceleration) < ACCELERATION_THRESHOLD:
                        rospy.loginfo("Publishing GPS data: %s", gps_msg)
                        gps_pub.publish(gps_msg)
                    else:
                        rospy.logwarn("Filtered GPS data due to high acceleration: %s m/s^2", acceleration)

                    # Update last velocity and time
                    last_velocity = velocity
                    last_time = current_time
            elif sentence.startswith("$GPHDT"):
                heading = parse_hdt(sentence)
                if heading is not None:
                    heading_msg = Float64(heading)
                    rospy.loginfo("Publishing HDT data: %s", heading_msg)
                    hdt_pub.publish(heading_msg)

if __name__ == '__main__':
    rospy.init_node('gps_node', anonymous=True)
    gps_pub = rospy.Publisher('/gps/gps_fix', NavSatFix, queue_size=10)
    hdt_pub = rospy.Publisher('/heading', Float64, queue_size=10)
    baseLat = -33.8474  # Example base station latitude
    baseLon = 150.651  # Example base station longitude
    try:
        read_gps_data()
    except rospy.ROSInterruptException:
        pass
