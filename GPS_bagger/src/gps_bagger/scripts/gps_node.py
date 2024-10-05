#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import serial
from util.parser import parse_gpgga, parse_hdt

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