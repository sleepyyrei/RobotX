#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import NavSatFix
from datetime import datetime
import uuid
import socket
import pytz  # For timezone support
import time  # Import time for sleep


class HeartPublisher:
    def __init__(self, server_hostname, server_port):
        rospy.init_node('heartbeat_publisher', anonymous=True)
        
        # Constants
        self.server_hostname = server_hostname  # Server hostname (socket ID)
        self.server_port = server_port          # Server port number
        self.team_id = "NTCH"                   # Team ID constant
        self.rate = rospy.Rate(1)               # 1 Hz

        # Initialize dynamic fields with default values
        self.latitude = 0.0
        self.longitude = 0.0
        self.ns_indicator = 'N'
        self.ew_indicator = 'W'
        self.system_mode = 2  # Default to Autonomous
        self.uav_status = 1   # Default to Stowed

        # Subscribers to required topics
        rospy.Subscriber('/gps/gps_fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('MODE', Int32, self.mode_callback)
        rospy.Subscriber('UAV', Int32, self.uav_callback)

    def gps_callback(self, msg):
        # Update GPS data from topic
        self.latitude = msg.latitude
        self.ns_indicator = 'N' if msg.latitude >= 0 else 'S'
        self.longitude = msg.longitude
        self.ew_indicator = 'E' if msg.longitude >= 0 else 'W'

    def mode_callback(self, msg):
        # Update system mode from MODE topic
        self.system_mode = msg.data

    def uav_callback(self, msg):
        # Update UAV status from UAV topic
        self.uav_status = msg.data

    def calculate_checksum(self, message):
        # Calculate the checksum by bitwise XOR over all characters in the message
        checksum = 0
        for char in message:
            checksum ^= ord(char)
        return f"{checksum:X}"  # Return as hexadecimal string

    def create_heartbeat_message(self):
        # Generate a unique message ID using UUID
        unique_message_id = f"${uuid.uuid4().hex[:8].upper()}"

        # Get the current date and time in U.S. Eastern Standard Time (EST)
        est = pytz.timezone('US/Eastern')
        current_time = datetime.now(est)
        est_date = current_time.strftime("%d%m%y")
        est_time = current_time.strftime("%H%M%S")

        # Format message without checksum
        msg_without_checksum = f"RXHRB,{est_date},{est_time},{self.latitude},{self.ns_indicator}," \
                               f"{self.longitude},{self.ew_indicator},{self.team_id},{self.system_mode},{self.uav_status}"
        
        # Calculate checksum
        checksum = self.calculate_checksum(msg_without_checksum)
        
        # Construct the complete message with checksum
        heartbeat_message = f"${msg_without_checksum}*{checksum}"
        
        return heartbeat_message
    
    def sender(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.server_hostname, self.server_port))
        
        for i in range(1000000000):
            heartbeat_message = self.create_heartbeat_message()
            print(f"Heartbeat Message: {heartbeat_message}")
            try:
                # Send the heartbeat message over TCP to the server
                self.sock.sendall(heartbeat_message.encode())
                rospy.loginfo(f"Sent heartbeat message: {heartbeat_message}")
            except socket.error as e:
                rospy.logerr(f"Socket error: {e}")
                rospy.signal_shutdown("Socket send error")
                break  # Exit the loop if there's a socket error

            time.sleep(1)  # Delay for 1 second

    def __del__(self):
        # Close the socket when the object is deleted or the node shuts down
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
            rospy.loginfo("Socket closed.")


if __name__ == '__main__':
    try:
        server_hostname = 'robot.server'
        server_port = 12345
        hb = HeartPublisher(server_hostname, server_port)
        hb.sender()
    except rospy.ROSInterruptException:
        pass