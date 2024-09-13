#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from util.parser import parse_gpgga
import serial
import time

def parse_gga_sentence(sentence):
    """Parse a $GNGGA sentence and return latitude, longitude, and altitude."""
    parts = sentence.split(',')
    try:
        lat = float(parts[2])  # Latitude in ddmm.mmmm
        lon = float(parts[4])  # Longitude in dddmm.mmmm
        alt = float(parts[9])  # Altitude in meters

        # Convert latitude and longitude to decimal degrees
        lat_deg = int(lat / 100)
        lat_min = lat % 100
        latitude = lat_deg + (lat_min / 60.0)

        lon_deg = int(lon / 100)
        lon_min = lon % 100
        longitude = lon_deg + (lon_min / 60.0)

        return latitude, longitude, alt
    except (IndexError, ValueError):
        rospy.logwarn("Failed to parse GNGGA sentence")
        return None, None, None

def parse_hdt_sentence(sentence):
    """Parse a $HDT sentence and return yaw information."""
    parts = sentence.split(',')
    try:
        yaw = float(parts[1])  # Heading in degrees
        return yaw
    except (IndexError, ValueError):
        rospy.logwarn("Failed to parse HDT sentence")
        return None
def gps_cmd_callback(data):
        global gps_cmd
        gps_cmd = data

def read_gps_data():
    # Replace '/dev/ttyUSB0' with your serial port
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    rospy.loginfo("Connected to GPS")

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('ascii', errors='replace').strip()

            if line.startswith('$GPGGA'):
                lat, lon, alt = parse_gpgga(line)
                if lat is not None and lon is not None:
                    gps_data = (lat, lon, alt)
                    if gps_data is not None:
                        lat, lon, alt = gps_data
                        gps_msg = NavSatFix()
                        gps_msg.header.stamp = rospy.Time.now()
                        gps_msg.header.frame_id = "gps"
                        gps_msg.latitude = lat
                        gps_msg.longitude = lon
                        gps_msg.altitude = alt

                        rospy.loginfo("Publishing GPS data: %s", gps_msg)
                        gps_pub.publish(gps_msg)
                        gps_data = None  # Reset after publishing

            elif line.startswith('$HDT'):
                yaw = parse_hdt_sentence(line)
                if yaw is not None:
                    rospy.loginfo("Yaw information: %f", yaw)
                    yaw_pub.publish(yaw)
                    yaw = None

            elif line.lower() in ["next"]:
                try:
                    latitude = gps_cmd.latitude
                    longitude = gps_cmd.longitude
                    altitude = gps_cmd.altitude

                # Format the data as a string
                    message = f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}"
                    ser.write((message + '\n').encode('ascii'))
                    global gps_last_cmd 
                    gps_last_cmd = gps_cmd
                except:
                    pass
            elif line.lower() in ["repeat"]:
                try:
                    latitude = gps_last_cmd.latitude
                    longitude = gps_last_cmd.longitude
                    altitude = gps_last_cmd.altitude

                # Format the data as a string
                    message = f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}"
                    ser.write((message + '\n').encode('ascii'))
                except:
                    pass

if __name__ == '__main__':
    gps_data = None
    gps_cmd = None
    gps_last_cmd = None
    rospy.init_node('gps_node', anonymous=True)
    gps_pub = rospy.Publisher('/gps/gps_prop', NavSatFix, queue_size=10)
    yaw_pub = rospy.Publisher('/gps/yaw', Float32, queue_size=10)
    rospy.Subscriber('/gps/gps_cmd', NavSatFix, gps_cmd_callback) 

    try:
        read_gps_data()
    except rospy.ROSInterruptException:
        pass
