###########################################################
## GPS based navigator + Geofencing + Visual navigation  ##
## Author: SHUBHAM SHARMA                                ##
## Copyright: Rebhu Computing Pvt Ltd                    ##
###########################################################

import py_trees as pt
import sys
import time
import operator
from threading import Thread
from .utils.config_parser import Config
from .utils.aux import Interface, Sensors
from .utils.generic_functions import *
from .navigation.aux import NavigationData, check_geofence, person_detection
# =====================================================
# IMPORT BEHAVIOURS
# =====================================================
from .trees.sequence1 import SystemCheck
from .trees.sequence2 import CheckGPSData, CheckIMUData, SensorDataWriter, GeofenceDataWriter, WaypointsDataWriter
from .trees.sequence3 import GeofenceGuard, WaypointNavigation, WaypointsDataFlush

# =====================================================
# CREATE ROOT
# =====================================================

def create_root(config, interface, sensors, navigation_data, run_type) -> pt.behaviour.Behaviour:
    """
    This function creates the Root Behaviour and it's Sub-Trees
    
    Returns: 
        the root behaviour
    """
    
    # SET BLACKBOARD FOR KEY-VALUES
    
    # ROOT
    root = pt.composites.Sequence(name="Root", memory=False)
    
    # SYSTEM CHECK
    system_check = SystemCheck(name="SystemCheck", config=config, interface=interface, sensors=sensors)
    one_shot = pt.decorators.OneShot(name="OneShot", child=system_check, policy=pt.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)
    
    # DATA WRITER
    guard_data_and_write = pt.composites.Sequence(name="Guard Data and Write", memory=False)
    # ----- ETERNAL GUARD FOR SENSOR DATA
    data_guard = pt.composites.Sequence(name="Guard?", memory=False)
    # ---------- GPS GUARD
    check_gps = CheckGPSData(name="Check GPS", sensors=sensors)
    # ---------- IMU GUARD
    check_imu = CheckIMUData(name="Check IMU", sensors=sensors)
    
    # ----- BLACKBOARD WRITER FOR KEY-VALUE UPDATES ALONG WITH AN ETERNAL GUARD
    data_writer = pt.composites.Sequence(name="Write", memory=False)
    # ---------- SENSOR WRITER
    write_sensor_data = SensorDataWriter(name="Sensor Data Write", sensors=sensors)
    # ---------- NAVIGATION WRITER
    write_navigation_data = pt.composites.Sequence(name="Navigation Data Write", memory=False)
    # --------------- GEOFENCE WRITE
    write_geofence_data = GeofenceDataWriter(name="Geofence Data Write", navigation_data=navigation_data)
    # --------------- WAYPOINT WRITE
    write_waypoints_data = WaypointsDataWriter(name="Waypoint Data Write", navigation_data=navigation_data)
    
    # NAVIGATION
    navigation = pt.composites.Sequence(name="Navigation", memory=False)
    
    # ----- GEOFENCE GUARD
    geofence_guard = GeofenceGuard(name="Geofence?", interface=interface, can_config=config.user.interface.CAN, ros_config=config.user.interface.ROS2, run_type=run_type)
    
    # ----- VISUAL
    visual_navigation = pt.composites.Sequence(name="Visual", memory=False)
    # ---------- PERSON DETECTION GUARD
    person_detection_guard = pt.behaviours.Success(name="Person?")
    # person_detection_guard = pt.decorators.EternalGuard(name="Person?", 
    #                                                     condition=person_detection,
    #                                                     child=send_kill)
    # ----- WAYPOINT NAVIGATION
    waypoint = pt.composites.Sequence(name="Waypoint", memory=False)
    # ---------- BATTERY CHECK GUARD
    battery_check = pt.behaviours.Success(name='Battery Check?')
    # ---------- WAYPOINT TRIGGER
    waypoint_trigger = pt.behaviours.CheckBlackboardVariableValue(name='Waypoint Trigger', 
                                                                  check=pt.common.ComparisonExpression(
                                                                      variable="runtime/navigation/waypoints/trigger",
                                                                      value=True,
                                                                      operator=operator.eq
                                                                  ))
    # ---------- WAYPOINT NAVIGATION
    waypoint_navigation = WaypointNavigation(name="Waypoint Navigation", 
                                             sensors=sensors, 
                                             interface=interface, 
                                             run_type=run_type)
    # waypoint_navigation = pt.behaviours.Running(name="Waypoint Navigation")
    # ---------- FLUSH WAYPOINT DATA
    flush_waypoints = WaypointsDataFlush(name="Flush Waypoints")
    
    # RECOVERY
    recovery =  pt.behaviours.Success(name='Recovery')
    
    # ASSEMBLE TREE
    root.add_children([one_shot, guard_data_and_write, navigation, recovery])
    guard_data_and_write.add_children([data_guard, data_writer])
    data_guard.add_children([check_gps, check_imu])
    data_writer.add_children([write_sensor_data, write_navigation_data])
    write_navigation_data.add_children([write_geofence_data, write_waypoints_data])
    navigation.add_children([geofence_guard, person_detection_guard, waypoint])
    waypoint.add_children([battery_check, waypoint_trigger, waypoint_navigation, flush_waypoints])
    
    
    
    return root, waypoint_navigation, geofence_guard

class ACU:
    """
    Mother class responsible to Initiate the:
    * Interface
    * Sensors
    * Behaviour Tree
    """
    
    def __init__(self, run_type='real', gps_type='rtk', logging=False):
        # set log level
        if logging:
            pt.logging.level = pt.logging.Level.DEBUG
        self.isStopped = False
        self.waypoint_navigation = None
        self.logging = logging
        self.run_type = run_type
        
        # load configuration parameters =======================================================
        path = "/home/debian/ros2_ws/src/autonomy_si/autonomy_si/ocular2/app/"
        self.config = Config(path+'app_config.yaml', path+'user_config.yaml')
        # ====================================================================================
        
        # Initial setup, sets Interface (CAN), Sensors (GPS and IMU) ==========================
        self.interface = Interface(self.config.app.interface.CAN, self.config.user.interface.CAN, interface_type='can', run_type=run_type)
        
        if run_type == 'real':
            self.sensors = Sensors(self.config.app.sensors, run_type='real', gps_type=gps_type)
            if not self.sensors.success:
                print("Sensors not loaded. Exiting...")
                self.interface.shutdown()
                sys.exit()
            self.sensors.start()
        else:
            self.sensors = Sensors(self.config.app.sensors, run_type='simulation', gps_type=gps_type)
        # ====================================================================================
        
        # Navigation data ====================================================================
        self.navigation_data = NavigationData()
        # ====================================================================================
    
    def _register_keys(self, blackboard, key, access_type):
        blackboard.register_key(key=key, access=access_type)
        
    
    def set_blackboard(self):
        """
        Function to create blackboards with two namespaces one 
        for permanent variables and another for runtime variables
        """
        
        # set blackboard stream for debug purposes
        # pt.blackboard.Blackboard.enable_activity_stream(maximum_size=1000)
        
        # init two blackboard clients
        blackboard_app = pt.blackboard.Client(name="Configuration App", namespace="app")
        self.blackboard_user = pt.blackboard.Client(name="Configuration User", namespace="user")
        blackboard_runtime = pt.blackboard.Client(name="Configuration Runtime", namespace="runtime")
        
        # APP
        for key in iterate_nested_dict(self.config.APP):
            # register keys
            self._register_keys(blackboard_app, key, pt.common.Access.WRITE)

            # register values
            blackboard_app.set(key, get_value_from_nested_key(self.config.APP, key))
        
        # USER
        for key in iterate_nested_dict(self.config.USER):

            # register keys
            self._register_keys(self.blackboard_user, key, pt.common.Access.WRITE)

            # register values
            self.blackboard_user.set(key, get_value_from_nested_key(self.config.USER, key))
       
        # RUNTIME
        for key in ['sensors/gps/latitude', 'sensors/gps/longitude', 'sensors/imu/heading']:
            self._register_keys(blackboard_runtime, key, pt.common.Access.READ)
        
        # ------ NAVIGATION
        self._register_keys(blackboard_runtime, 'navigation/geofence', pt.common.Access.READ)
        self._register_keys(blackboard_runtime, 'navigation/waypoints', pt.common.Access.READ)
        self._register_keys(blackboard_runtime, 'navigation/waypoints/trigger', pt.common.Access.READ)
        self._register_keys(blackboard_runtime, 'navigation/waypoints/repeat', pt.common.Access.WRITE)
        self._register_keys(blackboard_runtime, 'navigation/waypoints/pause', pt.common.Access.WRITE)
        
        
        # set defaults for runtime variables
        blackboard_runtime.set("navigation/waypoints/repeat", False)
        blackboard_runtime.set("navigation/waypoints/pause", False)
        
    
    def update_blackboard(self, user_config):
        self.config.USER = user_config
        
        # USER
        try:
            for key in iterate_nested_dict(self.config.USER):
                # register values
                self.blackboard_user.set(key, get_value_from_nested_key(self.config.USER, key), overwrite=True)
            
            return True
        except:
            return False
    
    def update_blackboard_variable(self, namespace, variable):
        try:
            blackboard_client = pt.blackboard.Client(name="Configuration", namespace=namespace)
            self._register_keys(blackboard_client, variable["key"], pt.common.Access.WRITE)
            blackboard_client.set(variable["key"], variable["value"])
        except:
            return False
        return True
    
    def _latlon2lonlat(self, latlon):
        return [[x[1], x[0]] for x in latlon]
    
    def _extract_waypoints(self, waypoints):
        new_wp = []
        for wp in waypoints:
            new_wp.append([wp['longitude'], wp['latitude'], wp['id'], wp['pre_timeout'], wp['post_timeout'], wp['action'], wp['turn']])
        return new_wp
    
    def nav_data(self, geofence=None, waypoints=None):
        
        # The data comes in format [lat, lon] but the application 
        # requires it to be in [lon, lat] format, so applying a filter.
        
        if geofence:
            self.navigation_data.GEOFENCE = self._latlon2lonlat(geofence)
        
        if waypoints:
            self.navigation_data.WAYPOINTS = self._extract_waypoints(waypoints)
        
        
    def start(self):
        Thread(target=self.tree, args=()).start()
        return self
        
    
    
    def tree(self):
        root, self.waypoint_navigation, self.geofence_guard = create_root(config=self.config, 
                                                    interface=self.interface, 
                                                    sensors=self.sensors, 
                                                    navigation_data=self.navigation_data,
                                                    run_type=self.run_type)
        # pt.display.render_dot_tree(root, with_blackboard_variables=True)
        root.setup_with_descendants()
        
        ss = 0
        while not self.isStopped:
            # print(ss)
            ss+=1
            time.sleep(1)
            root.tick_once()
            if self.logging:
                print(pt.display.unicode_blackboard())
            
        self.waypoint_navigation.terminate(pt.common.Status.SUCCESS)
        
        # ------------ PRINT STATEMENTS ----------------------------------------
        # print(pt.display.unicode_tree(root, show_status=True))
        # print("--------------------------\n")
        # print(pt.display.unicode_blackboard())
        # print("--------------------------\n")
        # print(pt.display.unicode_blackboard(display_only_key_metadata=True))
        # print(pt.display.unicode_blackboard_activity_stream())
        # ----------------------------------------------------------------------
    
    def stop_waypoints_navigation(self):
        self.waypoint_navigation.terminate(pt.common.Status.SUCCESS)
    
    def waypoints_navigation_status(self, stype="navigation"):
        if stype == "navigation":
            if self.geofence_guard.status == pt.common.Status.FAILURE:
                return "successful | Geofence violation"
            
            if self.waypoint_navigation.status == pt.common.Status.RUNNING:
                return "running | {}".format(self.waypoint_navigation.msg)
            elif self.waypoint_navigation.status == pt.common.Status.SUCCESS:
                return "successful | {}".format(self.waypoint_navigation.msg)
            elif self.waypoint_navigation.status == pt.common.Status.FAILURE:
                return "failed | {}".format(self.waypoint_navigation.msg)
            else:
                if self.waypoint_navigation.msg == 'stopped':
                    return "successful | stopped"
                return "Idle | None"
        
        elif stype == "action":
            return self.waypoint_navigation.msg_action

    def shutdown(self):
        try:
            self.waypoint_navigation.terminate(pt.common.Status.SUCCESS)
        except:
            pass
        self.isStopped = True
        self.interface.shutdown()
        self.sensors.stop()
    
# if __name__ == "__main__":
#     acu = ACU()
#     acu.tree()
#     acu.shutdown()9997260452