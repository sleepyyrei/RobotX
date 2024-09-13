#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import serial
from util.parser import parse_gpgga

# def parse_gpgga(gpgga_message):
#     """
#     Parses a GPGGA message to extract latitude, longitude, and altitude.

#     Parameters:
#     gpgga_message (str): The GPGGA message string.

#     Returns:
#     tuple: A tuple containing latitude (float), longitude (float), and altitude (float).
#     """

#     # Split the GPGGA message by commas
#     parts = gpgga_message.split(',')

#     # Extract latitude and longitude
#     lat_raw = parts[2]
#     lat_dir = parts[3]
#     lon_raw = parts[4]
#     lon_dir = parts[5]

#     # Convert latitude to decimal degrees
#     lat_deg = float(lat_raw[:2])
#     lat_min = float(lat_raw[2:])
#     lat = lat_deg + (lat_min / 60.0)
#     if lat_dir == 'S':
#         lat = -lat

#     # Convert longitude to decimal degrees
#     lon_deg = float(lon_raw[:3])
#     lon_min = float(lon_raw[3:])
#     lon = lon_deg + (lon_min / 60.0)
#     if lon_dir == 'W':
#         lon = -lon

#     # Extract altitude
#     alt = float(parts[9])

#     return lat, lon, alt

def read_gps_data():
    # Replace '/dev/ttyUSB0' with your serial port
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    rospy.loginfo("Connected to GPS")

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

                    rospy.loginfo("Publishing GPS data: %s", gps_msg)
                    gps_pub.publish(gps_msg)

if __name__ == '__main__':
    rospy.init_node('gps_node', anonymous=True)
    gps_pub = rospy.Publisher('gps/gps_fix', NavSatFix, queue_size=10)
    try:
        read_gps_data()
    except rospy.ROSInterruptException:
        pass
