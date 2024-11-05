from util.maps import Pose, Obstacle
from util.NavChannel import NavChannel
from util.obstacleTracker import ObstacleTracker
from gps_bagger.msg import Obstacles
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from gps_bagger.srv import WaypointService, WaypointServiceRequest
import math


class ChannelNavigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('channel_navigator', anonymous=True)
        
        # Create a subscriber for the custom message
        self.obstacles_sub = rospy.Subscriber('/obstacles', Obstacles, self.obstacles_callback)
        self.gps_sub = rospy.Subscriber('/gps/gps_fix', NavSatFix, self.gps_callback)
        self.heading_sub = rospy.Subscriber('/heading', Float64, self.heading_callback)
        self.gps_received = False
        self.heading = 0
        self.heading_received = False
        self.wait_for_initial_messages()
        
        self.obstacle_tracker = ObstacleTracker(current_pose=self.current_pose)
        self.path_planner = NavChannel()

        # Log that the subscriber is ready
        rospy.loginfo("ChannelNaviator is running")

    def dynamic_reconfig_callback(self, config, level):
        self.obstacle_tracker.dynamic_reconfig_callback(config, level)
        return
    def wait_for_initial_messages(self):
        # Wait for the first GPS and heading messages
        while not rospy.is_shutdown() and not (self.gps_received and self.heading_received):
            rospy.sleep(0.1)  # Sleep briefly to avoid busy waiting

    def obstacles_callback(self, msg):
        # Process the received message
        rospy.loginfo("Received obstacles data:")
        obstacles = []
        for i in range(len(msg.nav_sat_data)):
            gps_data = msg.nav_sat_data[i]
            # obstacles.append(Obstacle(lat= gps_data.latitude, lon=gps_data.longitude, 
            #                           origin = self.current_pose, heading = 0, 
            #                           colour = msg.colour[i], object_type=msg.type[i]))
            obstacles.append(Obstacle(x= msg.x[i], y=msg.y[i], 
                                      origin = self.current_pose, heading = 0, 
                                      colour = msg.colour[i], object_type=msg.type[i]))
        current_pose = self.obstacle_tracker.current_pose
        self.obstacle_tracker.obstacle_tracking(obstacles=obstacles)
        
        red_bouys = []
        black_bouys = []
        blue_bouys = []
        start_bouy = None
        end_bouy = None
        for obstacle in self.obstacle_tracker.obstacles:
            if obstacle.pose.object_type == "bouy" and obstacle.pose.colour == "red":
                red_bouys.append(obstacle.pose)
            elif obstacle.pose.object_type == "bouy" and obstacle.pose.colour == "blue":
                blue_bouys.append(obstacle.pose)
            elif obstacle.pose.object_type == "bouy" and obstacle.pose.colour == "black":
                black_bouys.append(obstacle.pose)
            elif obstacle.pose.object_type == "start_bouy":
                start_bouy = obstacle.pose
            elif obstacle.pose.object_type == "end_bouy":
                end_bouy = obstacle.pose
        path = self.path_planner.getPath(red_bouys=red_bouys, blue_bouys=blue_bouys, 
                                         start_bouy=start_bouy, end_bouy=end_bouy, obstacles=black_bouys)
        
        path_poses = []
        prev_pose = path[0]
        path.pop(0)
        for pose in path:
            yaw = ChannelNavigator.angle_from_vertical(pose[0]-prev_pose[0], pose[1] - prev_pose[1])
            path_poses.append(Pose(x=pose[0], y=pose[1], origin=current_pose, yaw=yaw, heading=None))
            prev_pose = pose
        self.waypoint_service_client(poses=path_poses)

    def angle_from_vertical(x, y):
        # Calculate the angle in radians from the vertical
        angle_rad = math.atan2(x, y)

        # Convert to degrees
        angle_deg = math.degrees(angle_rad)

        # Adjust the angle to be in the range [0, 360)
        angle_deg = (angle_deg + 360) % 360

        return angle_deg


    def waypoint_service_client(self, poses):
        """Call the WaypointService with the given waypoints and headings."""
        rospy.wait_for_service('/waypoint_service')  # Wait until the service is available
        try:
            waypoint_service = rospy.ServiceProxy('/waypoint_service', WaypointService)
            request = WaypointServiceRequest()
            waypoints = []
            headings = []
            for pose in poses:
                waypoint = NavSatFix()
                waypoint.latitude = pose.lat
                waypoint.longitude = pose.lon
                waypoints.append(waypoint)
                headings.append(pose.heading)
            request.waypoints = waypoints  # Assign waypoints to the request
            request.headings = headings     # Assign headings to the request
            response = waypoint_service(request)  # Call the service
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def gps_callback(self, msg):
        self.current_pose = Pose(lat=msg.latitude, lon=msg.longitude, x=0, y=0, yaw=0, heading=self.heading)
        if self.gps_received == True:
            self.obstacle_tracker.gps_callback(msg)
        self.gps_received = True
        return


    def heading_callback(self, msg):
        self.heading = msg.data
        if self.heading_received == True:
            self.obstacle_tracker.heading_callback(msg)
        self.heading_received = True

        return

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_subscriber = ChannelNavigator()
        obstacle_subscriber.spin()
    except rospy.ROSInterruptException:
        pass