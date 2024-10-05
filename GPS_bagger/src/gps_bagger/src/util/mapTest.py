import unittest
from typing import Optional
from common_utilities import enu_to_gps, gps_to_enu, convert_cartesian
from maps import Map

# Assuming the Pose class is defined as shown in your code snippet
class TestPose(unittest.TestCase):
    def setUp(self):
        """
        Create an initial origin Pose for testing.
        """
        # Define an origin Pose with known GPS coordinates and heading
        self.origin = Map.Pose(lat=37.7749, lon=-122.4194, x=0.0, y=0.0, origin=None, heading=0)

    def test_pose_with_gps_to_xy(self):
        """
        Test Pose transformation from GPS to ENU (x, y) coordinates.
        """
        # Create a Pose with known GPS coordinates
        pose = Map.Pose(lat=37.7750, lon=-122.4195, x=None, y=None, origin=self.origin, heading=10.0, yaw=None)
        
        # Verify that the x and y coordinates have been calculated
        self.assertIsNotNone(pose.x, "x coordinate should be calculated")
        self.assertIsNotNone(pose.y, "y coordinate should be calculated")
        
        # Print the results for debugging purposes
        print(f"Calculated x: {pose.x}, Calculated y: {pose.y}")
        
        # Check the heading and yaw
        self.assertAlmostEqual(pose.heading, 10.0, places=3, msg="Heading should match")
        self.assertAlmostEqual(pose.yaw, -10.0, places=3, msg="Yaw should match origin heading difference")

    def test_pose_with_xy_to_gps(self):
        """
        Test Pose transformation from ENU (x, y) to GPS coordinates.
        """
        # Create a Pose with known x, y coordinates
        pose = Map.Pose(lat=None, lon=None, x=10.0, y=20.0, origin=self.origin, heading=45.0, yaw=None)
        
        # Verify that the lat and lon coordinates have been calculated
        self.assertIsNotNone(pose.lat, "Latitude should be calculated")
        self.assertIsNotNone(pose.lon, "Longitude should be calculated")
        
        # Print the results for debugging purposes
        print(f"Calculated lat: {pose.lat}, Calculated lon: {pose.lon}")
        
        # Check the heading and yaw
        self.assertAlmostEqual(pose.heading, 45.0, places=3, msg="Heading should match")
        self.assertAlmostEqual(pose.yaw, -45.0, places=3, msg="Yaw should match origin heading difference")

    # def test_tolerance_check(self):
    #     """
    #     Test the tolerance check between calculated and reference coordinates.
    #     """
    #     # Create a Pose with known GPS coordinates and calculated x, y
    #     pose = Map.Pose(lat=37.7750, lon=-122.4195, x=1.0, y=1.0, origin=self.origin)
        
    #     # Check if the x and y coordinates are within tolerance
    #     self.assertTrue(pose.check_tolerance(1.05, pose.x, tolerance=0.1), "x coordinate should be within tolerance")
    #     self.assertTrue(pose.check_tolerance(1.05, pose.y, tolerance=0.1), "y coordinate should be within tolerance")
        
    #     # Print for debugging purposes
    #     print(f"X tolerance: {pose.check_tolerance(1.05, pose.x)}")
    #     print(f"Y tolerance: {pose.check_tolerance(1.05, pose.y)}")
    #     print(gps_to_enu(lat=37.7750, lon=-122.4195,alt=0,baseLat=37.7749, baseLon=-122.4194))
    def test_rotation(self):
        """
        Test the tolerance check between calculated and reference coordinates.
        """
        self.origin.heading = 280
        # Create a Pose with known GPS coordinates and calculated x, y
        pose = Map.Pose(x=5, y=0, heading=-10, origin=self.origin, yaw=None)
        
        print(pose.lat, ',', pose.lon, ',', pose.x, ',', pose.y)

        new_origin = Map.Pose(x=-1,y=-10,yaw=-10,origin=self.origin,heading=None)
        print("origin:" ,new_origin.lat, ',', new_origin.lon,  ',', new_origin.heading)
        new_origin.x = 0
        new_origin.y = 0
        new_origin.yaw = 0
        pose.changeOrigin(new_origin=new_origin, copy=True)
        print(pose.lat, ',', pose.lon, ',', pose.x, ',', pose.y)


if __name__ == '__main__':
    unittest.main()
