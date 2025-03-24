import py_trees as pt
from shapely.geometry import Polygon


class CheckGPSData(pt.behaviour.Behaviour):
    """
    This class is an Eternal Guard for checking GPS data
    """
    
    def __init__(self, name: str, sensors: None):
        """
        Init the node
        Args:
            name: behaviour name
            sensors: sensors object
        """
        super(CheckGPSData, self).__init__(name)
        self.gps = sensors.GPS
        
    def update(self):
        # check gps data
        if self.gps.has_fix:
            new_status = pt.common.Status.SUCCESS
            self.logger.debug("{} [UPDATE] [{} -> {}]".format(self.__class__.__name__, new_status, "GPS Data available"))
            return new_status
        else:
            new_status = pt.common.Status.FAILURE
            self.logger.error("{} [UPDATE] [{} -> {}]".format(self.__class__.__name__, new_status, "GPS Data not available"))
            return new_status
        
        
class CheckIMUData(pt.behaviour.Behaviour):
    """
    This class is an Eternal Guard for checking IMU data
    """
    
    def __init__(self, name: str, sensors: None):
        """
        Init the node
        Args:
            name: behaviour name
            sensors: sensors object
        """
        super(CheckIMUData, self).__init__(name)
        self.imu = sensors.IMU
        
    def update(self):
        # check gps data
        if self.imu.has_fix:
            new_status = pt.common.Status.SUCCESS
            self.logger.debug("{} [UPDATE] [{} -> {}]".format(self.__class__.__name__, new_status, "IMU Data available"))
            return new_status
        else:
            new_status = pt.common.Status.FAILURE
            self.logger.error("{} [UPDATE] [{} -> {}]".format(self.__class__.__name__, new_status, "IMU Data not available"))
            return new_status
 
 

class SensorDataWriter(pt.behaviour.Behaviour):
    """
    This class stores variables in a key-value store:
    and update Blackboard variables with sensor values e.g. Latitude, Longitude and Heading
    """
    def __init__(self, name: str, sensors):
        """
        Init the blackboard
        
        Args: 
            name: behaviour name
            sensors: sensor object
        """
        super(SensorDataWriter, self).__init__(name)
        
        # init a blackboard client
        self.blackboard = self.attach_blackboard_client("Configuration Runtime", "runtime")        
        
        # register keys
        for key in ['sensors/gps/latitude', 'sensors/gps/longitude', 'sensors/imu/heading']:
            self.blackboard.register_key(key=key, access=pt.common.Access.WRITE)
        
        self.logger.debug("%s.__init__() [INIT]" % (self.__class__.__name__))
        
        self.sensors = sensors
        
    def update(self):
        # write lat lon heading to the board

        self.blackboard.sensors.gps.latitude = self.sensors.GPS.lat
        self.blackboard.sensors.gps.longitude = self.sensors.GPS.lon
        self.blackboard.sensors.imu.heading = self.sensors.IMU.heading
        
        return pt.common.Status.SUCCESS


class GeofenceDataWriter(pt.behaviour.Behaviour):
    """
    This class stores variables in a key-value store:
    and update Blackboard variables with Geofence Data Latitude, Longitude
    """
    def __init__(self, name: str, navigation_data):
        """
        Init the blackboard
        
        Args: 
            name: behaviour name
            navigation_data: navigation data
        """
        super(GeofenceDataWriter, self).__init__(name)
        
        # init a blackboard client
        self.blackboard = self.attach_blackboard_client("Configuration Runtime", "runtime")
        
        # register keys
        self.blackboard.register_key(key='navigation/geofence', access=pt.common.Access.WRITE)
        
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.navigation_data = navigation_data
        
    def update(self):
        
        if self.navigation_data.GEOFENCE:
            self.blackboard.navigation.geofence = Polygon(self.navigation_data.GEOFENCE)
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
        

class WaypointsDataWriter(pt.behaviour.Behaviour):
    """
    This class stores variables in a key-value store:
    and update Blackboard variables with Waypoints Data Latitude, Longitude
    """
    def __init__(self, name: str, navigation_data):
        """
        Init the blackboard
        
        Args: 
            name: behaviour name
            navigation_data: navigation data
        """
        super(WaypointsDataWriter, self).__init__(name)
        
        # init a blackboard client
        self.blackboard = self.attach_blackboard_client("Configuration Runtime", "runtime")
        
        # register keys
        self.blackboard.register_key(key='navigation/waypoints', access=pt.common.Access.WRITE)
        self.blackboard.register_key(key='navigation/waypoints/trigger', access=pt.common.Access.WRITE)
        
        self.logger.debug("%s.__init__() [INIT]" % (self.__class__.__name__))
        self.navigation_data = navigation_data
        
    def update(self):
        
        if self.navigation_data.WAYPOINTS:
            self.blackboard.navigation.waypoints = self.navigation_data.WAYPOINTS
            self.blackboard.set('navigation/waypoints/trigger', True)
            self.navigation_data.WAYPOINTS = None
            
        return  pt.common.Status.SUCCESS
        
