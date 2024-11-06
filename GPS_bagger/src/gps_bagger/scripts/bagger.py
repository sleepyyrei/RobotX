#!/usr/bin/env python3

import rospy
import subprocess
from datetime import datetime
from gps_bagger.srv import callResponse, callResponseResponse
import os
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
                return callResponseResponse(response="Started recording")
            else:
                return callResponseResponse(response="Already recording")
        elif req.command == "stop":
            if self.recording_process is not None:
                # Stop recording
                self.stop_recording()
                return callResponseResponse(response="Stopped recording")
            else:
                return callResponseResponse(response="Not recording")
        else:
            return callResponseResponse(response="Invalid command")

    def start_recording(self):
        # Generate a timestamp for the filename
        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        filename = os.path.expanduser(f"~/Rei_WS/rosbags/bag_{timestamp}.bag")  # Save in workspace directory

        # Start recording with the generated filename
        self.recording_process = subprocess.Popen(['rosbag', 'record', '-O', filename, 
                                                '/gps/gps_fix', 
                                                '/mavros/global_position/global', 
                                                '/mavros/imu/data', 
                                                '/mavros/setpoint_raw/global'])
        rospy.loginfo(f"Started recording to {filename}")


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
