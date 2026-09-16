from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time
import math
import geopy.distance
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import logging
import os
import shutil
from datetime import datetime, timezone


class MAVLinkHandlerDronekit:
    """
    init by: mavlink_handler = MAVLinkHandler(f'127.0.0.1:{port}')
    """
    def __init__(self, connection_string, _wait_ready=False, refresh_rate=50, logs=False, prod=False, do_log=False):
        if not logs:
            logging.getLogger('dronekit').setLevel(logging.CRITICAL)
        if prod:
            print('Setting permissions for /dev/ttyACM0')
            os.system("sudo chmod a+rw /dev/ttyACM0")

        self.master = connect(connection_string, wait_ready=_wait_ready, rate=refresh_rate)
        self.boot_time = time.time()

        self.do_log = do_log
        if self.do_log:
            self.init_logger()

    def init_logger(self):
        # Customcustom logger in order to log to both console and file
        self.logger = logging.Logger('mavlinkHandler')
        # Set the log level
        self.logger.setLevel(logging.DEBUG)
        # Create handlers
        c_handler = logging.StreamHandler()
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        log_file_path = f"mavlink_{timestamp}.log"
        old_logs_dir = "mavlink_logs"
        
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            # Generate a unique name for the log file in the old logs directory
            timestamp = datetime.datetime.now()
            new_log_file_name = log_file_path
            new_log_file_path = os.path.join(old_logs_dir, new_log_file_name)
            # Move the log file to the old logs directory
            shutil.move(log_file_path, new_log_file_path)
            
        f_handler = logging.FileHandler(log_file_path)
        # Set levels for handlers
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add it to handlers
        f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        f_handler.setFormatter(f_format)

        # Add handlers to the self.logger
        self.logger.addHandler(f_handler)

    def arm_vehicle(self):
        self.master.armed = True

    def disarm_vehicle(self):  
        self.master.armed = False

    def takeoff(self):
        self.master.mode = VehicleMode("TAKEOFF")

    def set_mode(self, mode_name):
        self.master.mode = VehicleMode(mode_name)

    def set_parameter_value(self, parameter_name, value):
        self.master.parameters[parameter_name] = value

    def set_target_attitude(self, roll, pitch, yaw, thrust, roll_rate=0, pitch_rate=0, yaw_rate=0, throttle_ignore=False):
        msg = self.master.message_factory.set_attitude_target_encode(
            int(1e3 * (time.time() - self.boot_time)),
            self.master._master.target_system, self.master._master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE if throttle_ignore else 0,
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            roll_rate, 
            pitch_rate, 
            yaw_rate, 
            thrust
        )
        if self.do_log:
            self.logger.info(f'Sending set_attitude_target message with roll: {roll}, pitch: {pitch}, yaw: {yaw}, thrust: {thrust}')
        self.master.send_mavlink(msg)

    def get_attitude(self):
        if self.do_log:
            self.logger.info(f"Got Roll: {self.master.attitude.roll}, Pitch: {self.master.attitude.pitch}, Yaw: {self.master.attitude.yaw}")
        return math.degrees(self.master.attitude.roll), math.degrees(self.master.attitude.pitch), math.degrees(self.master.attitude.yaw)
    
    
    def get_battery(self):
        """
        Returns a tuple with the battery voltage and the battery level
        Returns:
            tuple: (voltage, level)
        """
        voltage, level = self.master.battery.voltage, self.master.battery.level
        if self.do_log:
            self.logger.info(f"Got Battery Voltage: {voltage}, Level: {level}")
        return voltage, level

    def get_lidar_distance(self):
        distance = self.master.rangefinder.distance
        if self.do_log:
            self.logger.info(f"Got LiDAR Distance: {distance}")
        return distance

    def get_mode_mapping(self):
        mode_mapping = self.master._mode_mapping
        if self.do_log:
            self.logger.info(f"Got Mode Mapping: {mode_mapping}")
        return mode_mapping

    def get_parameter_value(self, parameter_name):
        value = self.master.parameters[parameter_name]
        if self.do_log:
            self.logger.info(f"Got Parameter '{parameter_name}' Value: {value}")
        return value

    def get_mode(self):
        mode_name = self.master.mode.name
        if self.do_log:
            self.logger.info(f"Got Mode: {mode_name}")
        return mode_name

    def get_heading(self):
        heading = self.master.heading
        if self.do_log:
            self.logger.info(f"Got Heading: {heading}")
        return heading

    def get_air_speed(self):
        airspeed = self.master.airspeed
        if self.do_log:
            self.logger.info(f"Got Air Speed: {airspeed}")
        return airspeed

    def get_ground_speed(self):
        groundspeed = self.master.groundspeed
        if self.do_log:
            self.logger.info(f"Got Ground Speed: {groundspeed}")
        return groundspeed

    def get_location(self):
        lat = self.master.location.global_relative_frame.lat
        lon = self.master.location.global_relative_frame.lon
        alt = self.master.location.global_relative_frame.alt
        if self.do_log:
            self.logger.info(f"Got Location: Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
        return lat, lon, alt

    def simple_go_to(self, lat, lon, alt, block=False, distance_radius=15):
        a_location = LocationGlobalRelative(lat, lon, alt)
        self.master.simple_goto(a_location)

        if self.do_log:
            self.logger.info(f"Simple Go To: Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, Blocking: {block}, Distance Radius: {distance_radius}")

        if block:
            while True:
                time.sleep(0.1)
                vehicle_lat, vehicle_lon, vehicle_alt = self.get_location()
                point1 = (vehicle_lat, vehicle_lon)
                target = (lat, lon)
                distance = geopy.distance.geodesic(target, point1).meters
                if self.do_log:
                    self.logger.info(f"Current Location: {vehicle_lat}, {vehicle_lon}, {vehicle_alt}, Distance to Target: {distance} meters")
                if distance <= distance_radius:
                    if self.do_log:
                        self.logger.info(f"Arrived at Target Location: {lat}, {lon}, {alt}")
                    break

    def clear_mission(self):
        cmds = self.master.commands
        cmds.clear()
        self.master.commands.upload()

    def readmission(self, aFileName):
        cmds = self.master.commands
        missionlist=[]
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def upload_mission(self, file_path):
        missionlist = self.readmission(file_path)
        cmds = self.master.commands
        cmds.clear()

        for command in missionlist:
            cmds.add(command)

        self.master.commands.upload()

    def gps_time(self):
        message = self.master._master.recv_match(type='SYSTEM_TIME', blocking=True).to_dict()
        utc_time = datetime.fromtimestamp(message["time_unix_usec"] / 1e6, tz=timezone.utc)
        return utc_time.hour, utc_time.minute, utc_time.second, utc_time.microsecond // 1000
            
    

class MAVLinkHandlerPymavlink:
    def __init__(self, connection_string, _autoreconnect=False, message_hz=50, prod=False):
        if prod:
            print('Setting permissions for /dev/ttyACM0')
            os.system("sudo chmod a+rw /dev/ttyACM0")

        self.master = mavutil.mavlink_connection(connection_string, autoreconnect=_autoreconnect)
        self.master.wait_heartbeat()
        self.boot_time = time.time()

        used_messages = [mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT]
        self.request_message_interval(used_messages, message_hz)
    
    def request_message_interval(self, message_list, frequency_hz):
        for message_input in message_list:
            self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_input,
            1e6 / frequency_hz,
            0, 0, 0, 0, 0)

    def set_target_attitude(self, roll, pitch, yaw, thrust, roll_rate=0, pitch_rate=0, yaw_rate=0, throttle_ignore=False):
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE if throttle_ignore else 0,
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            roll_rate, 
            pitch_rate, 
            yaw_rate, 
            thrust
        )

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.set_mode(mode_id)

    def get_attitude(self):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if msg is not None:
            return math.degrees(msg.pitch), math.degrees(msg.roll), math.degrees(msg.yaw)
        
    def simple_goto(self, lat , lon, alt, block=False, distance_radius=15):

        self.master.mav.mission_item_send(
            self.master.target_system, self.master.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, 0, 0, 0,
            lat, lon, alt
        )
        if block:
            while True:
                time.sleep(0.1)
                point1 = self.get_location()
                target = (lat, lon)
                distance = geopy.distance.geodesic(target, point1[:2]).meters
                if distance <= distance_radius:
                    break

    def get_location(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            return msg.lat/1.0e7, msg.lon/1.0e7, msg.alt/1000.0

    def get_mode(self):
        message = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if message:
            mode = mavutil.mode_string_v10(message)
            return mode
        else:
            print("No HEARTBEAT message received")

    def arm_vehicle(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 to arm, 0 to disarm
        )
        print(f"Vehicle armed")


if __name__ == '__main__':
    """Test"""
    mavlink_handler = MAVLinkHandlerDronekit('udp:127.0.0.1:14591',_wait_ready=False)
    while True:
        print(mavlink_handler.gps_time())
        time.sleep(1)