import py_trees as pt

# System Check Guard
class SystemCheck(pt.behaviour.Behaviour):
    """
    This class acts as a guard at the entry gate for Navigation.
    This Sub-Tree runs with OneShot decorator, to make sure peripherals are up before Nav activation
    
    """
    def __init__(self, name: str, config: None, interface: None, sensors: None):
        """
        Init the Guard
        
        Args: 
            name: behaviour name
        """
        super(SystemCheck, self).__init__(name)
        self.config = config
        self.interface = interface
        self.sensors = sensors
        self.logger.debug("{}.__init__()".format(self.__class__.__name__))
        
        
    def setup(self):
        """
        Setup sequence for system level checks.
        * real or Driver initiliation
        * Integration layer initilisation (e.g. ROS pub/sub)
        
        Note: Currently Empty
        """
        
        self.logger.debug("Setting up system level checks")

    def initialise(self) -> None:
        """
        Initial setup for updating parameters or variables necessary for running this behaviour
        This method is called on first tick and everytime the status is not RUNNING
        
        Note: Currently returning None    
        """
        self.logger.debug("Initilisation done for system level checks")
         
    def update(self) -> pt.common.Status:
        """
        Runs actual system checks after setup and initialisation.
        The checks may include but not limited to:
        * CAN interface
        * GPS Sensor
        * IMU Sensor
        
        Note: THIS SHOULD BE NON-BLOCKING
        """
        # check CAN
        self.logger.info("{} [Update] {}".format(self.__class__.__name__, "Checking Interface"))
        if not self.interface.isLive():
            new_status = pt.common.Status.FAILURE
            self.logger.error("{} [Update] {} {}".format(self.__class__.__name__, new_status, "Interface not live"))
            return new_status

        # check sensors
        self.logger.info("{} [Update] {}".format(self.__class__.__name__, "Checking Sensors"))
        if not self.sensors.isLive()[0]:
            new_status = pt.common.Status.FAILURE
            self.logger.error("{} [Update] {} {} -> {}".format(self.__class__.__name__, new_status, "Sensors not active", self.sensors.isLive()[1]))
            return new_status
        
        
        self.logger.debug("{} [Update] {}".format(self.__class__.__name__, self.status))
        return pt.common.Status.SUCCESS
    
    def terminate(self, new_status: pt.common.Status) -> None:
        """
        This method is called whenever the behaviour switches to a non RUNNING status
        """
        self.logger.debug("{} [TERMINATING] [{}]".format(self.__class__.__name__, self.status))
