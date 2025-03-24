import yaml

class AppInterfaceConfig:
    """ For Interface """
    def __init__(self, interface_config):
        self.CAN = interface_config['can0']

class UserInterfaceConfig:
    """ For Interface """
    def __init__(self, interface_config):
        self.CAN = interface_config['can0']
        self.ROS2 = interface_config['ros2']
        
class SensorsConfig:
    """ For Sensors """
    def __init__(self, interface_config):
        self.GPS = interface_config['gps0']
        self.RTK = interface_config['rtk0']
        self.IMU = interface_config['imu0']

class NavigationConfig:
    """ For Geofence """
    def __init__(self, interface_config):
        self.GEOFENCE = interface_config['geofence']
        self.VISUAL = interface_config['visual']
        self.WAYPOINTS = interface_config['waypoints']



class Permanent:
    """
    Class to load permanent configuration parameters.
    * Interface
        * CAN
    * Sensors
        * GPS
        * IMU
    Args:
        config_dict
    """
    def __init__(self, config_dict):
        
        # load interface parameters
        self.interface = AppInterfaceConfig(config_dict['interface'])
        
        # load sensor parameters
        self.sensors = SensorsConfig(config_dict['sensors'])

class User:
    """
    Class to load User defined configuration parameters.
    * Interface
        * CAN
    * Sensors
        * GPS
        * IMU
    Args:
        config_dict
    """
    def __init__(self, config_dict):
        
        # load navigation parameters
        self.navigation = NavigationConfig(config_dict['navigation'])
        
        # load interface parameters
        self.interface = UserInterfaceConfig(config_dict['interface'])
        self.INTERFACE = config_dict['interface']
        

class Config:
    """
    Class to load configuration parameters.
    Args:
        config_file (YAML)
    """
    def __init__(self, app_config_file, user_config_file):
        
        # open app config file
        with open(app_config_file, 'r') as fl:
            try:
                self.app_config_dict = yaml.safe_load(fl)
                self.success = True        # check after parsing the class
            
            except yaml.YAMLError as exc:
                self.success = False        # check after parsing the class
        
        # open user config file
        with open(user_config_file, 'r') as fl:
            try:
                self.user_config_dict = yaml.safe_load(fl)
                self.success = True        # check after parsing the class

            except yaml.YAMLError as exc:
                self.success = False        # check after parsing the class
                

        # load permanent class of parameters
        self.app = Permanent(self.app_config_dict)
        self.APP = self.app_config_dict
        
        # load user defined parameters
        self.user = User(self.user_config_dict)
        self.USER = self.user_config_dict