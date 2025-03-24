import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
from std_msgs.msg import String
from .app.navigation_bt import ACU
from threading import Thread

class ACUController:
    def __init__(self, run_type='real', gps_type='rtk', logging=True):
        self.status = 'killed'
        self.acu = ACU(run_type, gps_type, logging=logging)
        self.status = 'setup'
        self.logging = logging
        
    def init(self):
        self.acu.set_blackboard()
        self.status = 'init'
    
    def start(self):
        self.acu.start()
        self.status = 'running'
        
    def kill(self):
        self.acu.shutdown()
        self.status = 'killed'
    
    def reset(self, run_type='real', gps_type='rtk'):
        if self.status != 'killed':
            self.kill()   
        self.acu = ACU(run_type=run_type, gps_type=gps_type, logging=self.logging)
        # time.sleep(2)
        self.status = 'setup'
        self.init()
        # time.sleep(5)
        self.start()

            
class ACUControlNode(Node):
    """
    Parent class for running control listener ACU
    """
    def __init__(self, topics, ACUController):
        """
        Args:
            topics: dictionary of topics
            ACUController: ACU controlling object
        """
        super().__init__('acu_control_node')
        
        self.run_type = 'real'
        self.gps_type = 'rtk'
        self.acu_control = ACUController(run_type=self.run_type, gps_type=self.gps_type)
        self.acu_control_msg = None
        self.waypoints = None
        self.waypoints_status = False
        self.call_once = True
        
        self.acu_control_subscription = self.create_subscription(String, topics["acu_control"], self.acu_control_listener_callback, 1)
        self.acu_control_gps_switch = self.create_subscription(String, topics["acu_control_gps"], self.acu_control_gps_listener_callback, 1)
        
        self.acu_navigation_missionfile_subs = self.create_subscription(String, topics["acu_navigation_missionfile"], self.missionfile_listener_callback, 1)

        self.acu_waypoints_control_subs = self.create_subscription(String, topics["acu_waypoints_control"], self.waypoints_controller_callback, 1)
        self.acu_user_config_subs = self.create_subscription(String, topics["acu_user_config"], self.user_config_controller_callback, 1)
        
        self.acu_sensors_subscription = self.create_subscription(String, topics["sensors"], self.sensors_callback, 1)
        
        self.acu_navigation_waypoint_publisher = self.create_publisher(Twist, topics["acu_navigation_waypoints_control"], 1)
        self.create_timer(0.1, self.acu_navigation_waypoint_callback)
        
        self.acu_navigation_waypoint_action_publisher = self.create_publisher(String, topics["acu_navigation_waypoints_action"], 1)
        
        self.acu_upload_publisher = self.create_publisher(String, topics["acu_upload_status"], 1)
        
        self.waypoints_navigation_status_publisher = self.create_publisher(String, topics["waypoints_navigation_status"], 1)
        self.create_timer(1, self.acu_navigation_waypoint_status_callback)
        
        self.waypoints_navigation_cords_publisher = self.create_publisher(String, topics["waypoints_navigation_coords"], 1)
        self.create_timer(0.5, self.acu_navigation_waypoint_cords_callback)
        
    def acu_control_listener_callback(self, msg):
        if msg.data == 'kill':
            self.acu_control.kill()
            
        elif msg.data == 'reset':
            self.run_type = 'real'
            self.acu_control.reset(run_type=self.run_type, gps_type=self.gps_type)
            
        elif msg.data == 'real':
            self.run_type = 'real'
            self.acu_control.reset(run_type='real')
            
        elif msg.data == 'simulation':
            self.run_type = 'simulation'
            self.acu_control.reset(run_type='simulation')

    def acu_control_gps_listener_callback(self, msg):
        if msg.data == 'gps':
            self.gps_type = 'gps'
        elif msg.data == 'rtk':
            self.gps_type = 'rtk'

    
    def missionfile_listener_callback(self, msg):
        
        try:
            data = json.loads(msg.data)
            self.acu_control.acu.nav_data(geofence=data["Missionfile"]["Actiondata"]["geofence"])
            
            # check waypoints for all keys
            wp_data = data["Missionfile"]["Actiondata"]["waypoints"]
            
            wp_flag = False
            for wp in wp_data:
                if not all(key in wp for key in ['longitude', 'latitude', 'id', 'pre_timeout', 'post_timeout', 'action', 'turn']):
                    wp_flag = False
                    break
                else:
                    wp_flag = True
            
            if wp_flag:
                self.waypoints = wp_data
                # update blackboard variable
                self.acu_control.acu.update_blackboard_variable("runtime", {"key": "navigation/waypoints/repeat", 
                                                                            "value": data["Missionfile"]["Actiondata"]["repeat"]})
                self.push_acu_upload_status(True, "MissionFile updated")
            
            else:
                self.push_acu_upload_status(False, "MissionFile not updated, WP key error")
    
        except:
            self.push_acu_upload_status(False, "MissionFile not updated")
    
    def waypoints_controller_callback(self, msg):
        if msg.data == 'start':
            if not self.waypoints_status:
                if self.waypoints:
                    self.acu_control.acu.nav_data(waypoints=self.waypoints)
                    self.waypoints_status = True
        elif msg.data == 'stop':
            if self.waypoints_status:
                self.waypoints = None
                self.acu_control.acu.stop_waypoints_navigation()
                self.waypoints_status = False
        elif msg.data == 'pause':
            self.acu_control.acu.update_blackboard_variable("runtime", {"key": "navigation/waypoints/pause", 
                                                                        "value": True})
        elif msg.data == 'resume':
            self.acu_control.acu.update_blackboard_variable("runtime", {"key": "navigation/waypoints/pause", 
                                                                        "value": False})
        
    def sensors_callback(self, msg):
        if self.run_type == 'simulation':
            try:
                data = json.loads(msg.data)
                if not (data["latitude"] == '' or data["heading"] == ''):
                    self.acu_control.acu.sensors.GPS.parse_gps(float(data["latitude"]), float(data["longitude"]))
                    self.acu_control.acu.sensors.IMU.parse_imu(float(data["heading"]))
            except:
                pass
            
 
    def acu_navigation_waypoint_callback(self):
        try:
            twist_msg = Twist()
            twist_msg.linear.x = self.acu_control.acu.waypoint_navigation.navigator.controls.cmd_vel["linear"]["x"]
            twist_msg.angular.z = self.acu_control.acu.waypoint_navigation.navigator.controls.cmd_vel["angular"]["z"]
            self.acu_navigation_waypoint_publisher.publish(twist_msg)
        except:
            pass
        
    def user_config_controller_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.acu_control.acu.update_blackboard(data)
            self.push_acu_upload_status(True, "config updated")
        except:
            self.push_acu_upload_status(False, "config not updated")
        
    def push_acu_upload_status(self, success, msg):
        payload = String()
        
        if success:
            data = {"status": "ok", 
                            "type": "upload", 
                            "message": msg
                }
        else:
            data = {"status": "failed", 
                            "type": "upload", 
                            "message": msg
                }
            
        payload.data = json.dumps(data)
        self.acu_upload_publisher.publish(payload)   
        
    def acu_navigation_waypoint_status_callback(self):
        payload = String()
        try:
            if self.acu_control.acu.waypoints_navigation_status().split(" |")[0] in ["successful", "failed"]:                
                self.waypoints_status = False
            elif self.acu_control.acu.waypoints_navigation_status().split(" |")[0] == "running":
                self.waypoints_status = True
            
            payload.data = self.acu_control.acu.waypoints_navigation_status()
            self.waypoints_navigation_status_publisher.publish(payload)
            
            msg = self.acu_control.acu.waypoints_navigation_status(stype="action")
            if msg:
                payload.data = msg
                self.acu_navigation_waypoint_action_publisher.publish(payload)
            else:
                payload.data = "None"
                self.acu_navigation_waypoint_action_publisher.publish(payload)
        except:
            pass
    
    def acu_navigation_waypoint_cords_callback(self):
        payload = String()
        try:
            payload.data = json.dumps({"heading": self.acu_control.acu.sensors.IMU.heading, 
                            "latitude": self.acu_control.acu.sensors.GPS.lat, 
                            "longitude": self.acu_control.acu.sensors.GPS.lon})
            
            self.waypoints_navigation_cords_publisher.publish(payload)
        except:
            pass
        
    
class Ros:
    def __init__(self):
        self.rclpy = rclpy
        self.rclpy.init(args=None)
        self.executor = rclpy.executors.MultiThreadedExecutor()

    
    
    def spin(self, node):
        Thread(target=self.run, args=(node, )).start()
        return self
    
    def run(self, node):
        self.rclpy.spin(node)


def main(args=None):
    run_type='simulation'
    topics = {  "acu_control": "acu_control",
                "acu_control_gps": "acu_control_gps",
                "acu_navigation_geofence": "acu_navigation_geofence", 
                "acu_navigation_waypoints": "acu_navigation_waypoints", 
                "acu_navigation_waypoints_action": "acu_navigation_waypoints_action", 
                "acu_navigation_missionfile": "acu_navigation_missionfile",
                "acu_waypoints_control": "acu_waypoints_control",
                "acu_user_config": "acu_user_config", 
                "sensors": "position_data", 
                "acu_upload_status": "acu_upload_status", 
                "waypoints_navigation_status": "waypoints_navigation_status", 
                "waypoints_navigation_coords": "waypoints_navigation_coords", 
                "acu_navigation_waypoints_control": "cmd_vel_out"
                }
    
    # Init RCLPY class
    ross = Ros()
    
    # ACU Controller Node
    acu_controller_node = ACUControlNode(topics, ACUController)
    
    # Pass Nodes
    ross.spin(acu_controller_node)
    
    acu_controller_node.acu_control.init()
    acu_controller_node.acu_control.start()

        
    # acu_controller_node.destroy_node()
    # rclpy.shutdown()
    

# if __name__ == "__main__":
#     topics = {"listener":
#                 {"acu_navigation_control": "acu_navigation_control"},
#                 {"sensors": "location_data"}}
#     main('simulation', topics)