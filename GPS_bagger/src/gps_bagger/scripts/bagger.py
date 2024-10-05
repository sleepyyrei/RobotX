#!/usr/bin/env python3

import rospy
import subprocess
from gps_bagger.srv import callResponse

class Bagger:
    def __init__(self):
        self.recording_process = None
        rospy.init_node('bagger')

        # Initialize service
        self.start_stop_service = rospy.Service('bagger', callResponse, self.handle_service)
        rospy.loginfo("Service ready to start/stop rosbag recording")

    def handle_service(self, req):
        rospy.loginfo("Received request: %s", req.command)
        if req.command == "start":
            if self.recording_process is None:
                # Start recording
                self.start_recording()
                return baggerResponse(response="Started recording")
            else:
                return baggerResponse(response="Already recording")
        elif req.command == "stop":
            if self.recording_process is not None:
                # Stop recording
                self.stop_recording()
                return baggerResponse(response="Stopped recording")
            else:
                return baggerResponse(response="Not recording")
        else:
            return baggerResponse(response="Invalid command")

    def start_recording(self):
        # Define the command to start rosbag recording
        self.recording_process = subprocess.Popen(['rosbag', 'record', '-O', '/bag', '/gps/gps_fix', '/gps/gps_cmd'])
        rospy.loginfo("Started recording")

    def stop_recording(self):
        if self.recording_process:
            # Send SIGINT to stop the rosbag process
            self.recording_process.send_signal(subprocess.signal.SIGINT)
            self.recording_process.wait()  # Wait for the process to terminate
            rospy.loginfo("Stopped recording")
            self.recording_process = None

if __name__ == '__main__':
    Bagger()
    rospy.spin()
