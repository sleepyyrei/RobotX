from typing import Optional, cast
# from util.common_utilities import enu_to_gps, gps_to_enu, convert_cartesian
from common_utilities import enu_to_gps, gps_to_enu, convert_cartesian


class Pose:
    def __init__(self, 
                    lat: Optional[float]=None, # Latitude of current pose
                    lon: Optional[float]=None, # Longitude of current pose
                    x: Optional[float]=None, # Points in direction of ship's bow
                    y: Optional[float]=None, # Points in direction of ship's starboard (right) side
                    origin: Optional['Pose']=None, # Frame of reference from which x-y coordinates are based
                    heading: Optional[float] = 0, # Angle of pose relative to north (clockwise is positive)
                    yaw: Optional[float] = 0): # Angle of pose relative to origin (counterclockwise is positive)
        self.lat = lat 
        self.lon = lon
        self.x = x
        self.y = y
        self.origin = origin
        self.heading = heading
        self.yaw = yaw
        self.tolerance = 0.1  # Define tolerance value, adjust as needed
         
        if origin is None or origin.isValidOrigin() == False:
            return
        # Calculate yaw based on heading and origin heading
        if self.heading is not None:
            if origin.heading is not None:
                test_yaw = round(origin.heading - self.heading, 3)
                if self.yaw is not None:
                    if not self.check_tolerance(self.yaw, test_yaw, self.tolerance):
                        print(f"Warning: Yaw does not match calculated yaw. Input yaw: {self.yaw}, Calculated yaw: {test_yaw}")
                else:
                    self.yaw = test_yaw           
        # Calculate heading based on yaw and origin heading
        elif self.yaw is not None: 
            if origin.heading is not None:
                if self.heading is None:
                    self.heading = round(origin.heading - self.yaw, 3)
                elif self.heading is not None:
                    if not self.check_tolerance(self.heading, origin.heading - self.yaw, self.tolerance):
                        print(f"Warning: Heading does not match calculated heading. Input heading: {self.heading}, Calculated heading: {origin.heading - self.yaw}")
            elif self.heading is None:
                self.heading = -round(self.yaw, 3)

        # Calculate euclidean coordinates base on GPS coordinates
        if self.hasGPSCoords() and self.origin is not None:
            origin = cast(Pose, self.origin)
                
            # Perform the coordinate transformation using GPS to ENU
            test_e, test_n, _ = gps_to_enu(self.lat, self.lon, 0, origin.lat, origin.lon)
            
            # Transform coordinates relative to base axis
            if origin.heading is not None:
                x_base, y_base = convert_cartesian(test_e, test_n, origin.x, origin.y, -origin.heading)
            else:
                # Handle the case where origin coordinates or heading are not available
                x_base, y_base = test_e, test_n
            
            # Round calculations to 3 decimal places
            x_base = round(x_base, 3)
            y_base = round(y_base, 3)

            # Check if x_base and y_base are within tolerance of self.x and self.y
            if self.hasXYCoords():
                if not self.check_tolerance(x_base, self.x, self.tolerance):
                    print(f"Warning: x coordinate does not match calculated x coordinate. Input x: {self.x}, Calculated x: {x_base}")
                if not self.check_tolerance(y_base, self.y, self.tolerance):
                    print(f"Warning: y coordinate does not match calculated y coordinate. Input y: {self.y}, Calculated y: {y_base}")
            else:
                self.y = x_base
                self.x = y_base
            
        elif self.x is not None and self.y is not None and self.origin is not None:
            origin = cast(Pose, self.origin)
            
            # Perform the coordinate transformation from origin to ENU
            test_e, test_n = convert_cartesian(self.y, self.x, origin.x, origin.y, origin.heading)
            
            # Transform coordinates from ENU to GPS
            if origin.lat is not None and origin.lon is not None and origin.heading is not None:
                lat_base, lon_base, _ = enu_to_gps(x=test_e, y=test_n, z=0, baseLat=origin.lat, baseLon=origin.lon)
            
            if self.lat is not None and self.lon is not None:
                if not self.check_tolerance(lat_base, self.lat, self.tolerance):
                    print(f"Warning: Latitude does not match calculated latitude. Input lat: {self.lat}, Calculated lat: {lat_base}")
                if not self.check_tolerance(lon_base, self.lon, self.tolerance):
                    print(f"Warning: Longitude does not match calculated longitude. Input lon: {self.lon}, Calculated lon: {lon_base}")
            else:
                self.lat = lat_base
                self.lon = lon_base

        
    
    def changeOrigin(self, new_origin: 'Pose', copy: Optional[bool] = False):
        if not new_origin.isValidOrigin():
            return None
        # Do the conversion via Cartesian transformations first
        transform_vector = Pose(lat=new_origin.lat, lon=new_origin.lon, heading=new_origin.heading, origin=self.origin, yaw=None)
        x_new, y_new = convert_cartesian(self.x, self.y, transform_vector.x, transform_vector.y, transform_vector.yaw)
        if transform_vector.yaw is not None and self.yaw is not None:
            yaw_new = self.yaw - transform_vector.yaw
        else:
            yaw_new = 0
        if self.hasGPSCoords():
            testPose = Pose(self.lat, self.lon,origin=new_origin,heading=self.heading, yaw=None)
            errorFlag = False
            if not Pose.check_tolerance(testPose.x, x_new, 0.01):
                print("change origin, GPS x", testPose.x, "Cartesian X", x_new)
                errorFlag = True
            if not Pose.check_tolerance(testPose.y, y_new, 0.01):
                print("change origin, GPS Y", testPose.y, "Cartesian Y", y_new)
                errorFlag = True
            if not Pose.check_tolerance(testPose.yaw, yaw_new, 0.0001):
                print("change origin, GPS Yaw", testPose.yaw, "Cartesian Yaw", yaw_new)
                errorFlag = True
            if errorFlag:
                return None
            elif copy:
                return self.copy_from(testPose)
            elif not copy:
                return testPose
        else:
            testPose = Pose(x=x_new,y=y_new,yaw=yaw_new,origin=new_origin)
            if copy:
                return self.copy_from(testPose)
            else:
                return testPose

            

    def copy_from(self, other: 'Pose'):
        """
        Copy all values from another Pose instance to the current instance.
        
        Args:
            other (Map.Pose): The Pose instance to copy values from.
        """
        if not isinstance(other, Pose):
            raise TypeError("Expected a Map.Pose instance")
            
        self.lat = other.lat
        self.lon = other.lon
        self.x = other.x
        self.y = other.y
        self.origin = other.origin
        self.heading = other.heading
        self.yaw = other.yaw
        
        # Optionally, you can also copy additional properties if needed
        self.tolerance = other.tolerance
            
    def isValidOrigin(self) -> bool:
        """
        Check pose is a valid origin. A valid origin must have GPS coordinates and have x, y and yaw set to 0

        Returns:
            bool: True if GPS coordinates are present and x,y and yaw are 0, False otherwise.
        """
        return self.hasGPSCoords() and self.x == 0 and self.y == 0 and self.yaw == 0
    
    def hasGPSCoords(self) -> bool:
        return self.lat is not None and self.lon is not None
    
    def hasXYCoords(self) -> bool:
        return self.x is not None and self.y is not None

    @staticmethod
    def check_tolerance(calculated_value: float, reference_value: float, tolerance: float = 0.1) -> bool:
        """
        Check if the calculated_value is within tolerance of reference_value.
        
        Args:
            calculated_value: The value to check.
            reference_value: The reference value.
            tolerance: The tolerance range.
        """
        return abs(calculated_value - reference_value) <= tolerance
    
    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Pose):
            return False
        return (Pose.check_tolerance(self.lat, value.lat, 0.0000001) and Pose.check_tolerance(self.lon, value.lon, 0.0000001) 
                and Pose.check_tolerance(self.x, value.x, 0.01) and Pose.check_tolerance(self.y, value.y, 0.01)
                and self.origin == value.origin and Pose.check_tolerance(self.heading, value.heading, 0.0000001)
                and Pose.check_tolerance(self.yaw, value.yaw, 0.0000001))

class Obstacle(Pose):
    def __init__(self, 
                    lat: Optional[float]=None,  # Latitude of obstacle
                    lon: Optional[float]=None,  # Longitude of obstacle
                    x: Optional[float]=None,  # Points in direction of ship's bow
                    y: Optional[float]=None,  # Points in direction of ship's starboard (right) side
                    origin: Optional['Pose']=None,  # Frame of reference from which x-y coordinates are based
                    heading: Optional[float] = 0,  # Angle of pose relative to north (clockwise is positive)
                    yaw: Optional[float] = 0,  # Angle of pose relative to origin (counterclockwise is positive)
                    colour: str = "white",  # Colour of the obstacle
                    object_type: str = "unknown"):  # Type of the obstacle (e.g. rock, buoy, etc.)
        
        # Initialize parent Pose class
        super().__init__(lat, lon, x, y, origin, heading, yaw)
        
        # Additional attributes for Obstacle class
        self.colour = colour
        self.object_type = object_type

    def sameObjectType(self, other:'Obstacle'):
        return isinstance(other, Obstacle) and self.object_type == other.object_type and self.colour == other.colour

    def __str__(self):
        # Extend Pose class's __str__ method to include obstacle-specific details
        pose_str = super().__str__()
        return f"{pose_str}, Object Type: {self.object_type}, Colour: {self.colour}"