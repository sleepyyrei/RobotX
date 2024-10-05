#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from util.common_utilities import enu_to_gps,gps_to_enu
from util.planning_module import PathPlanner,Mission,DubinsPath

def cartesian_distance(lon1, lat1, lon2, lat2):
    """
    Calculate the straight-line distance between two points 
    specified by longitude and latitude (disregarding altitude).

    :param lon1: Longitude of the first point in degrees
    :param lat1: Latitude of the first point in degrees
    :param lon2: Longitude of the second point in degrees
    :param lat2: Latitude of the second point in degrees

    :return: Distance between the two points in meters
    """
    # Convert degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    
    # Earth's radius in meters
    r = 6371000
    
    # Approximate distances in meters
    x1 = r * lon1 * math.cos(lat1)
    y1 = r * lat1
    x2 = r * lon2 * math.cos(lat2)
    y2 = r * lat2
    
    # Distance in meters
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    return distance

class Diamond:
    def __init__(self,radius):
        self.lat0 = None
        self.lon0 = None
        self.yaw = None
        self.lat = None
        self.lon = None
        self.last_lat = None
        self.last_lon = None
        self.speed = None
        self.time = rospy.Time.now()
        self.wps = None
        self.planner = None
        self.cmd_pub = rospy.Publisher('/gps/gps_cmd', NavSatFix, queue_size=10)
        pass
    def wp_init(self):
        yaw = self.yaw
        wp1 = Mission.Waypoint(enu_x= 0 + radius*math.sin(math.radians(yaw)), enu_y = 0 + radius * math.cos(math.radians(yaw)), heading=yaw + 90)
        wp2 = Mission.Waypoint(enu_x=wp1.pose.enu_x + radius*math.sin(math.radians(wp1.pose.heading)), enu_y=wp1.pose.enu_y + radius*math.cos(math.radians(wp1.pose.heading)), heading=wp1.pose.heading+90)
        wp3 = Mission.Waypoint(enu_x= 0 + radius*math.sin(math.radians(yaw+90)), enu_y = 0 + radius * math.cos(math.radians(yaw+90)), heading=yaw - 90)
        wp4 = Mission.Waypoint(enu_x=0,enu_y=0,heading=yaw)
        self.wps = [wp1, wp2, wp3, wp4]
        self.planner = PathPlanner(Mission.Path(self.wps), DubinsPath)

    def gps_callback(self,data):
        if isinstance(data, NavSatFix):
            if (self.lat0 != None  and self.lon0 != None):
                self.lat0 = data.latitude
                self.lon0 = data.longitude
                if (self.yaw != None):
                    self.wp_init()
            else:
                self.last_lat = self.lat
                self.last_lon = self.lon
                self.lat = data.latitude
                self.lon = data.longitude
                self.speed = cartesian_distance(self.last_lon, self.last_lat, self.lon, self,lat)/(self.time - rospy.Time.now())
                self.time = rospy.Time.now()
    def yaw_callback(self,data):
        if isinstance(data, Float32):
            if (self.yaw == None and self.lat != None):
                self.yaw = data
                self.wp_init()
            self.yaw = data

            
    def update_planner(self):
        x,y,_ = gps_to_enu(lat=self.lat, lon= self.lon,alt=0, baseLat=self.lat0, baseLon= self.lon0)
        mission_complete, heading_des, cte, wp_des = self.planner.update(Mission.Waypoint(enu_x=x, enu_y= y,heading=self.yaw), self.speed)
        lat, lon,_ = enu_to_gps(x=wp_des.enu_x, y=wp_des.enu_y, baseLat= self.lat0, baseLon=self.lon0)
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0
        self.cmd_pub.publish(msg)
        return mission_complete





if __name__ == '__main__':
    try:
        rospy.init_node('diamond', anonymous=True)
        planner = Diamond(10)
        rospy.Subscriber('/gps/gps_fix', NavSatFix, planner.gps_callback)
        rospy.Subscriber('/gps/yaw', NavSatFix, planner.yaw_callback) 
        rate = rospy.Rate(10)  # 10 Hz
        mission_complete = False

        while (not rospy.is_shutdown() or mission_complete):
            try:
                # Create a message
                mission_complete = planner.update_planner()
            except:
                pass

            

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

