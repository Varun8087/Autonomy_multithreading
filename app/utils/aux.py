import can
import serial
# import adafruit_gps
import board
import busio
import adafruit_bno055
import socket
from ublox_gps import UbloxGps
import base64
import time
from threading import Thread
import numpy as np
import os
import pyproj
import scipy.spatial.transform
from pyquaternion import Quaternion
from filterpy.kalman import KalmanFilter
import datetime as dts


class Can:
    """
    This class manages the can interface for ACU and VCU
    """
    
    def __init__(self, app_can_config, user_can_config, run_type):
        
        # try to conect to CAN
        self.bus = can.interface.Bus(bustype='socketcan',
                                     channel=app_can_config['channel'],
                                     bitrate=app_can_config['bitrate'])
        self.success = None
        self.user_can_config = user_can_config
        self.run_type = run_type
        
    def isLive(self):
        """
        Method to check live status of CAN. It checks for battery level messages, if message received return SUCCESS
        """
        if self.run_type == "real":
            resp = False
            for _ in range(10):
                msg = None
                try:
                    msg = self.bus.recv(timeout=0.5)
                except:
                    pass
                if msg:
                    if msg.arbitration_id == self.user_can_config['battery']['id']:
                        resp = True
                        break
        elif self.run_type == "simulation":
            resp = True
        return resp
            
    def can_send(self, id, message):
        msg = can.Message(arbitration_id=id,
                          data=[*message],
                          is_extended_id=False)

        try:
            self.bus.send(msg)
            self.success = True
        except can.CanError as err:
            self.success = False

    def shutdown(self):
        self.bus.shutdown()


class Interface:
    """
    Set ACU to VCU interface e.g. CAN
    """
    def __init__(self, app_config, user_config, interface_type='can', run_type='real'):
        """
        Init interface based on type
        """
        self.interface_type = interface_type
        if interface_type == 'can':
            
            self.CAN = Can(app_config, user_config, run_type)
        self.success = None
        
    def isLive(self):
        return self.CAN.isLive()
    
    def msg_send(self, id, message):
        self.CAN.can_send(id, message)
        self.success = self.CAN.success
        
    def shutdown(self):
        """
        shutdown Interface
        """
        try:
            if self.interface_type == 'can':
                self.CAN.shutdown()
                success = True
        except:
            success = False

class RTK:
    """
    Class to handle RTK-GPS data
    """
    def __init__(self, config):
        
        try:
            self.port_gps = serial.Serial(config['rtk_gps_serial_port'], baudrate=config['rtk_gps_baudrate'], timeout=config['rtk_gps_timeout'])
            self.port_rtk = serial.Serial(config['rtk_serial_port'], baudrate=config['rtk_baudrate'], timeout=config['rtk_timeout'])
            self.gps = UbloxGps(self.port_gps)
            
            # connet to NTRIP server
            # Create a TCP socket and connect to the NTRIP server
            self.ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ntrip_socket.connect((config['ntrip_host'], config['ntrip_port']))
            
            headers = ('GET /{} HTTP/1.1\r\n' +
                    'Host: {}\r\n' +
                    'Ntrip-Version: Ntrip/2.0\r\n' +
                    'User-Agent: NTRIP Python Client\r\n' +
                    'Authorization: Basic {}\r\n\r\n').format(config['ntrip_mountpoint'],
                                                                config['ntrip_host'],
                                                                base64.b64encode('{}:{}'.format(config['ntrip_username'],config['ntrip_password']).encode()).decode())
            self.ntrip_socket.send(headers.encode())  # Encode the headers before sending
            self.success = True

        except:
            self.success = False
            
        self.lat = None
        self.lon = None
        self.has_fix = False
        self.isGPSStopped = False
        self.isRTKStopped = False
        
    def start(self):
        self.run_gps()
        self.run_rtk()
        
    def run_gps(self):
        Thread(target=self.parse_gps).start()
        return self
    
    def run_rtk(self):
        Thread(target=self.parse_rtk).start()
        return self
        
    def parse_gps(self):
        while not self.isGPSStopped:
            try:
                coords = self.gps.geo_coords()
                if coords:
                    self.lat, self.lon, h_accu = round(coords.lat, 7), round(coords.lon, 7), round(coords.hAcc, 2)
                    if h_accu < 50:
                        self.has_fix = True
                    else:
                        self.has_fix = False
                else:
                    self.has_fix = False
            except:
                self.has_fix = False
            
    def parse_rtk(self):
        
        # Read data from the NTRIP server and send to the GPS module
        while not self.isRTKStopped:
            data = self.ntrip_socket.recv(1024)
            if data:
                self.port_rtk.write(data)
    
    def stop(self):
        self.isGPSStopped = True
        self.isRTKStopped = True

class GPS:
    """
    Class to handle GPS data, it could be from simulation or from the Sensor
    """
    def __init__(self, config, run_type, imu):
        self.run_type = run_type
        # self.counter = 0
        self.imu = imu
        # self.sensor_fusion = SensorFusion()
        
        if run_type == 'real':
            try:
                # Read serial port for GPS data
                uart = serial.Serial(config['gps_serial_port'], baudrate=config['gps_baudrate'], timeout=config['gps_timeout'])
                self.gps = adafruit_gps.GPS(uart, debug=False)
                
                # send gps parameters
                self.gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')         # to get only location data
                self.gps.send_command(bytes("PMTK220,{}".format(config['gps_refresh_rate']), encoding='utf-8'))         # refresh rate, for lower refresh rate increase gps_timeout
                self.success = True
            
            except:
                self.success = False
                
        self.lat = None
        self.lon = None
        self.has_fix = True
        self.isStopped = False
        
    
    def start(self):
        Thread(target=self.parse_gps, args=()).start()
        return self
    
    def parse_gps(self, lat=None, lon=None):
        """
        Get coordinates out of GPS stream
        """
        
        if self.run_type == 'real':
            while not self.isStopped:
                try:
                    self.gps.update()
                except:
                    pass
                if self.gps.has_fix:
                    # self.lat, self.lon = self.sensor_fusion.get_sensor_data(self.gps.longitude, self.gps.latitude, self.imu.accel_enu, self.imu.std_dev_east, self.imu.std_dev_north)
                    self.lat, self.lon = self.gps.latitude, self.gps.longitude

                self.has_fix = self.gps.has_fix
        elif self.run_type == 'simulation':
            
            self.lat, self.lon = lat, lon
            self.has_fix = True
            # print('----------------------------------------')
            # print(lat, lon)
            # print(self.lat, self.lon)
            # print('----------------------------------------')
        
        
    def stop(self):
        self.isStopped = True


class SelfCalibration:

    def __init__(self, imu, run_type):
        self.filename = '/home/debian/.local/autonomy_si/magnetometer_offsets_variation_during_travel.txt'
        self.flag = False
        self.flag_file = True
        self.imul = imu
        self.run_type = run_type

    def initialise(self):
        
        try:
            with open(self.filename,'r') as file:
                lines = file.readlines()
                if lines:
                    last_row = lines[-1].strip()
                    
        except:
            self.flag_file = False

        if not self.flag_file:
            self.imul.mode = adafruit_bno055.CONFIG_MODE
            self.imul.offsets_magnetometer = (623, -18, 204)
            self.imul.offsets_accelerometer = (-24, -51, -13)   #(0, 0, 0)
            self.imul.offsets_gyroscope = (-3, 2, 0)
            self.imul.radius_magnetometer = 660
            self.imul.radius_accelerometer = 1000
            self.imul.mode = adafruit_bno055.NDOF_MODE
            
        else:
            values = last_row.replace(')','').replace('(','').replace(',',' ').split('  ')
            mag_offsets = [int(x) for x in values[0:3]]
            mag_radius = int(values[3])
            accel_offsets = [int(x) for x in values[4:7]]
            accel_radius = int(values[7])
            gyro_offsets = [int(x) for x in values[8:]]
            mag_off = mag_offsets
            mag_rad = mag_radius
            accel_off = accel_offsets
            accel_rad = accel_radius
            gyro_off = gyro_offsets

            
            self.imul.mode = adafruit_bno055.CONFIG_MODE
            self.imul.offsets_magnetometer = (mag_off[0],mag_off[1],mag_off[2])  
            self.imul.offsets_accelerometer = (accel_off[0],accel_off[1],accel_off[2])   
            self.imul.offsets_gyroscope = (gyro_off[0],gyro_off[1],gyro_off[2])    
            self.imul.radius_magnetometer = mag_rad   
            self.imul.radius_accelerometer = accel_rad   
            self.imul.mode = adafruit_bno055.NDOF_MODE
        time.sleep(2)
        return self.imul

    def close(self):
        with open(self.filename,'w') as file1:
            file1.write('{}  {}  {}  {}  {}\n'.format(self.imul.offsets_magnetometer,self.imul.radius_magnetometer,self.imul.offsets_accelerometer, self.imul.radius_accelerometer,self.imul.offsets_gyroscope))


class IMU:
    """
    Class to handle IMU data, it could be from simulation or from the Sensor
    """
    def __init__(self, config, run_type):
        self.run_type = run_type
        # acceleration_east = []          #matrices to keep the acceleration value
        # acceleration_north = []
        # acceleration_up = []
        
        if self.run_type == 'real':
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.imut = adafruit_bno055.BNO055_I2C(i2c)     #BNO055_extended(i2c)
                
                # set calibration parameters
                try:
                    self.imu_calibration = SelfCalibration(self.imut, self.run_type)
                    self.imu = self.imu_calibration.initialise()
                except:
                    pass
                
                # read imu couple of times to activate it
                while True:
                    if self.imu.euler[0] == 0.0:
                        self.imu.euler
                    else:
                        break   
                
                
                # # Now collect the acceleration data to calculate the acceleration ENU biases
                # for kl in range(100):
                #     # Read imu quaternion and linear_acceleration data
                #     q = self.imu.quaternion
                #     accel_xyz = np.array(self.imu.linear_acceleration)
                    
                #     # Normalize the quaternion
                #     quat = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
                #     quat = quat/quat.norm

                #     # Construct the rotation matrix using the quaternion
                #     R = np.array(quat.rotation_matrix)              #this is the rotation matrix
                #     acc_east, acc_north, acc_up = R @ accel_xyz     #these are the acceleartion in ENU coordinates
                #     acceleration_east.append(acc_east)
                #     acceleration_north.append(acc_north)
                #     acceleration_up.append(acc_up)
                #     time.sleep(0.01)
                    
                # # These values will be used in the error covariance matrix Q
                # self.std_dev_east = np.sqrt(np.var(acceleration_east))
                # self.std_dev_north = np.sqrt(np.var(acceleration_north))
                # self.std_dev_up = np.sqrt(np.var(acceleration_up))
                self.success = True
                
            except:
                self.success = False

        
        self.accel_enu = None
        self.heading = None
        self.has_fix = False
        self.isStopped = False
        
        
            
    def start(self):  
        Thread(target=self.parse_imu, args=()).start()
        return self
 
    def parse_imu(self, heading=None):
        """
        Get heading out of IMU stream
        """
        if self.run_type == 'real':
            while not self.isStopped:
                time.sleep(0.005)
                # keep checking calibration
                if self.imu.calibration_status[3] >= 0:
                    self.has_fix = True
                else:
                    self.has_fix = False
                
                # # Read imu quaternion and linear_acceleration data
                # q = self.imu.quaternion
                # accel_xyz = np.array(self.imu.linear_acceleration)
                
                # # Normalize the quaternion
                # quat = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
                # quat = quat/quat.norm
                
                # # Construct the rotation matrix using the quaternion
                # R = np.array(quat.rotation_matrix)              #this is the rotation matrix
                # self.accel_enu = R @ accel_xyz     #these are the acceleartion in ENU coordinates

                try:
                    head = self.imu.euler[0] - 90.0
                    if  head < 0:
                        head += 360
                    elif head > 360:
                        head -= 360
                    self.heading = head
                except:
                    pass
                
        elif self.run_type == 'simulation':
            self.heading = heading
            self.has_fix = True
            
    
    def stop(self):
        self.isStopped = True
        time.sleep(1)
        if self.run_type == "real":
            self.imu_calibration.close()
            time.sleep(1)


class Sensors:
    """
    Class to handle sensors GPS and IMU
    """
    def __init__(self, config, run_type, gps_type):
        """
        Init GPS and IMU Sensors
        """
        self.run_type = run_type
        self.IMU = IMU(config.IMU, run_type)
        if run_type == 'real':
            if gps_type == "gps":
                self.GPS = GPS(config.GPS, run_type, self.IMU)
        
            elif gps_type == "rtk":
                self.GPS = RTK(config.RTK)    
        else:
            self.GPS = GPS(config.GPS, run_type, self.IMU)
            
        if run_type == 'real':
            if self.GPS.success and self.IMU.success:
                self.success = True
            else:
                self.success = False

    def isLive(self):
        if self.run_type == 'real':
            # GPS ----------------------------------
            for _ in range(10):
                if self.GPS.has_fix:
                    gps_resp = True
                    break
                else:
                    gps_resp = False
            
            # IMU ----------------------------------
            for _ in range(10):
                if self.IMU.imu.calibration_status[3] == 3:
                    imu_resp = True
                    break
                else:
                    imu_resp = False
                    
        elif self.run_type == 'simulation':
            # GPS ----------------------------------
            for _ in range(10):
                if self.GPS.has_fix:
                    gps_resp = True
                    break
                else:
                    gps_resp = False
            
            # IMU ----------------------------------
            for _ in range(10):
                if self.IMU.has_fix:
                    imu_resp = True
                    break
                else:
                    imu_resp = False
        
        if not imu_resp:
            return False, "IMU"
        elif not gps_resp:
            return False, "GPS"
        else:
            return True, "OK"
    
    def start(self):
        # start IMU thread
        self.IMU.start()
        # start GPS thread
        self.GPS.start()
    
    def stop(self):
        # stop GPS thread
        self.GPS.stop()
        # start IMU thread
        self.IMU.stop()

class Transforms:

    def __init__(self,lon_org, lat_org, alt_org):
        self.transformer = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        self.transformer2 = pyproj.Transformer.from_crs(
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            )
        self.x_org, self.y_org, self.z_org = self.transformer.transform( lon_org,lat_org,  alt_org,radians=False)
        rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
        rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

        self.rotMatrix = rot1.dot(rot3)
        self.ecef_org = np.array([[self.x_org,self.y_org,self.z_org]]).T

    def geodetic2enu(self,lon, lat, alt):
        
        x, y, z = self.transformer.transform( lon,lat,  alt,radians=False)
        
        vec=np.array([[ x-self.x_org, y-self.y_org, z-self.z_org]]).T    
    
        enu = self.rotMatrix.dot(vec).T.ravel()
        return enu.T

    def enu2geodetic(self,x,y,z):
        
        ecefDelta = self.rotMatrix.T.dot( np.array([[x,y,z]]).T )
        ecef = ecefDelta + self.ecef_org
        lon, lat, alt = self.transformer2.transform( ecef[0,0],ecef[1,0],ecef[2,0],radians=False)

        return [lon,lat,alt]

class SensorFusion:

    def __init__(self):
        #choose filename atomatically
        # base_filename = "/var/log/autonomy-si/logs_ENU_{}".format(dts.datetime.now().strftime("%Y-%m-%d|%H:%M:%S"))
        # file_extension = ".txt"
        # index = 1
                
        # while True:
        #     filename = f"{base_filename}{index}{file_extension}"
        #     if not os.path.exists(filename):
        #         break
        #     index += 1
        # self.filename = filename

        # #choose filename atomatically
        # base_filename1 = "/var/log/autonomy-si/logs_LLA_{}".format(dts.datetime.now().strftime("%Y-%m-%d|%H:%M:%S"))
        # index1 = 1
                
        # while True:
        #     filename1 = f"{base_filename1}{index1}{file_extension}"
        #     if not os.path.exists(filename1):
        #         break
        #     index1 += 1
        # self.filename1 = filename1

        # Initialise the parameters
        self.first_time = True

        self.lat_filtered = None
        self.lon_filtered = None
        
        self.std_gps = 3  #meters according to the Adafruit Ultimate GPS breakout datasheet

        self.kf = None
        self.geo_transform = None

        self.time_now = dts.datetime.now()
        self.dt = None

    #Kalman filter
    def KF(self,x0,R,Q,std_x=3,std_v=1):
        #global dt
        time_temp = dts.datetime.now()
        self.dt = (time_temp-self.time_now).total_seconds()
        self.time_now = time_temp

        kf = KalmanFilter(dim_x=4,dim_z=2,dim_u=4)
        kf.x = x0   #np.array(x0)
        kf.P = np.diag([std_x**2,std_v**2,std_x**2,std_v**2])
        kf.H = np.array([[1.,0.,0.,0.],
                        [0.,0.,1.,0.]])
        kf.F = np.array([[1.,self.dt,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,self.dt],
                        [0.,0.,0.,1.]])
        kf.B = np.array([[0.5*self.dt**2,0.,0.,0.],
                        [0.,self.dt,0.,0.],
                        [0.,0.,0.5*self.dt**2,0.],
                        [0.,0.,0.,self.dt]])
        kf.Q = Q
        kf.R = R
        return kf
    
    def get_sensor_data(self, lon, lat, acc_enu, std_dev_east, std_dev_north):

        acc_east,acc_north,acc_up = acc_enu    
        # Check if it the first instance and initialise the Kalman filter
        if self.first_time:
            # The initial paramters
            # x0 is in the format [x,v_x,y,v_y]
            x0 = np.array([0.,0.,0.,0.])
            Q = np.diag([std_dev_east,1**2,std_dev_north,1**2])
            R = np.diag([self.std_gps**2,self.std_gps**2])
            
            # Initialize the Kalman Filter
            self.kf = self.KF(x0=x0,R=R,Q=Q)

            # Initializes the transformation ENU<->LLA (Latitude,Longitude,Altitude)
            self.geo_transform = Transforms(lon,lat,0)

            self.first_time = False
        
        # This start the Kalman filtering as the vehicle moves
        else:
            # Calculate the time difference and accordingly set the state transition matrix F and the control matrix B
            time_temp = dts.datetime.now()
            self.dt = (time_temp-self.time_now).total_seconds()
            self.time_now = time_temp
            self.kf.F = np.array([[1.,self.dt,0.,0.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,self.dt],
                            [0.,0.,0.,1.]])
            self.kf.B = np.array([[0.5*self.dt**2,0.,0.,0.],
                            [0.,self.dt,0.,0.],
                            [0.,0.,0.5*self.dt**2,0.],
                            [0.,0.,0.,self.dt]])
            
            # Convert the unfiltered GPS sensor coordinates to ENU
            lon_enu,lat_enu,_ = self.geo_transform.geodetic2enu(lon,lat,0.)

                
            # Provide the control vector: [a_east,a_east,a_north,a_north]
            u = np.kron(np.array([acc_east,acc_north]),np.array([1.,1.]))
            # Provide the observed measurement GPS sensor coordinates in [lon,lat] format
            z = np.array([lon_enu,lat_enu])

            # Kalman Prediction
            self.kf.predict(u=u)

            # Kalman Update
            self.kf.update(z)
            lon_enu_filtered = self.kf.x[0]
            lat_enu_filtered = self.kf.x[2]

            # Convert the filtered ENU coordinates to LLA format
            self.lon_filtered, self.lat_filtered,_ = self.geo_transform.enu2geodetic(lon_enu_filtered,lat_enu_filtered,0.)      
            
            # Log the coordinates along with the respective data in two files each for ENU and LLA formats. 
            # Comment this out in case you don't need the logs
            # with open(self.filename,'a+') as file0 and open(self.filename1,'a+') as file1:
            #     file0.write('{}  {}  {}  {}  {}  {}  {}  {}  {}  {}  {}  {}\n'.format(lon_enu,lat_enu,lon_enu_filtered,lat_enu_filtered,altitude,acc_east,acc_north,acc_up,accel_xyz[0],accel_xyz[1],accel_xyz[2],self.time_now))
            #     file1.write('{}  {}  {}  {}  {}  {}  {}  {}  {}  {}  {}  {}\n'.format(lon,lat,self.lon_filtered,self.lat_filtered,altitude,acc_east,acc_north,acc_up,accel_xyz[0],accel_xyz[1],accel_xyz[2],self.time_now))
        
        # self.counter += 1
        # Return the status along with the filtered lon and lat
        return self.lat_filtered,self.lon_filtered#self.lon_filtered,self.lat_filtered