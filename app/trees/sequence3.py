import py_trees as pt
from ..navigation.waypoint.waypoint_navigation import Navigator
from ..navigation.waypoint.controls import Controls
import time
from shapely.geometry import Point


class GeofenceGuard(pt.behaviour.Behaviour):
    """
    This class checks the Geofence Violation
    """
    def __init__(self, name: str, interface, can_config, ros_config, run_type):
        """        
        Args: 
            name: behaviour name
        """
        super(GeofenceGuard, self).__init__(name)
        # init a blackboard client
        self.blackboard_runtime = self.attach_blackboard_client("Configuration Runtime", "runtime")
        self.blackboard_runtime.register_key(key='navigation/geofence', access=pt.common.Access.READ)
        self.blackboard_runtime.register_key(key='sensors/gps/latitude', access=pt.common.Access.READ)
        self.blackboard_runtime.register_key(key='sensors/gps/longitude', access=pt.common.Access.READ)
        self.blackboard_runtime.register_key(key='navigation/waypoints/trigger', access=pt.common.Access.WRITE)
        
        self.controls = Controls(interface, can_config, ros_config, run_type)
        
    def update(self):
        polygon = self.blackboard_runtime.get('navigation/geofence')
        if polygon.contains(Point(self.blackboard_runtime.get('sensors/gps/longitude'), 
                                  self.blackboard_runtime.get('sensors/gps/latitude'))):
            return pt.common.Status.SUCCESS
        else:
            self.blackboard_runtime.set('navigation/waypoints/trigger', False)
            self.controls.stop()
            
            return pt.common.Status.FAILURE

class WaypointNavigation(pt.behaviour.Behaviour):
    """
    This class runs the waypoint navigation algorithm
    and send a RUNNING status with periodic feedback until path is completed
    """
    def __init__(self, name: str, sensors, interface, run_type):
        """
        Init the Waypoint Navigation
        
        Args: 
            name: behaviour name
        """
        super(WaypointNavigation, self).__init__(name)

        self.sensors = sensors
        self.interface = interface    
        self.run_type = run_type
        self.msg = "init"
        
    def initialise(self):
        
        # init a blackboard client
        blackboard_runtime = self.attach_blackboard_client("Configuration Runtime", "runtime")
        blackboard_user = self.attach_blackboard_client("Configuration Runtime", "user")
        
        # register keys
        blackboard_runtime.register_key(key='navigation/waypoints', access=pt.common.Access.READ)
        blackboard_runtime.register_key(key='navigation/waypoints/repeat', access=pt.common.Access.READ)
        blackboard_runtime.register_key(key='navigation/waypoints/pause', access=pt.common.Access.WRITE)
        blackboard_user.register_key(key='navigation/waypoints', access=pt.common.Access.READ)
        blackboard_user.register_key(key='interface/can0', access=pt.common.Access.READ)
        blackboard_user.register_key(key='interface/ros2', access=pt.common.Access.READ)
        
        # set pause to false before init
        blackboard_runtime.set('navigation/waypoints/pause', False)
        
        self.navigator = Navigator(self.sensors, 
                                   blackboard_runtime.get('navigation/waypoints'), 
                                   self.interface, 
                                   blackboard_user.get('navigation/waypoints'), 
                                   blackboard_user.get('interface/can0'), 
                                   blackboard_user.get('interface/ros2'), 
                                   run_type=self.run_type,
                                   repeat=blackboard_runtime.get('navigation/waypoints/repeat'))
        self.navigator.start()
        self.blackboard_runtime = blackboard_runtime
        time.sleep(2)
    
    def update(self):
        if self.navigator.feedback == "running":
            self.navigator.update_pause(self.blackboard_runtime.get('navigation/waypoints/pause'))
            self.msg = self.navigator.msg
            self.msg_action = self.navigator.msg_action
            return pt.common.Status.RUNNING
        elif self.navigator.feedback == "finished":
            self.msg = self.navigator.msg
            return pt.common.Status.SUCCESS
        elif self.navigator.feedback == "failed":
            self.msg = self.navigator.msg
            return pt.common.Status.SUCCESS
    
    def terminate(self, new_status: pt.common.Status) -> None:
        try:
            # set pause to false before init
            self.blackboard_runtime.set('navigation/waypoints/pause', False)
            self.navigator.stop()
        except:
            pass

class WaypointsDataFlush(pt.behaviour.Behaviour):
    """
    This class flush variables in a key-value store:
    """
    def __init__(self, name: str):
        """
        Init the blackboard
        
        Args: 
            name: behaviour name
        """
        super(WaypointsDataFlush, self).__init__(name)
        
        # init a blackboard client
        self.blackboard = self.attach_blackboard_client("Configuration Runtime", "runtime")
        
        # register keys
        self.blackboard.register_key(key='navigation/waypoints/trigger', access=pt.common.Access.WRITE)
        
        self.logger.debug("%s.__init__() [INIT]" % (self.__class__.__name__))
        
    def update(self):
        
        self.blackboard.set('navigation/waypoints/trigger', False)
        return pt.common.Status.SUCCESS
        