import numpy as np
import redis
import datetime
import os
import atexit
from simple_pid import PID
from collections import deque
from statistics import mean
from scipy import stats
from mavlinkHandler import MAVLinkHandlerDronekit as MAVLinkHandler
import json
import time 
import threading
import logging
import shutil

from behav_control import adjust_angles

class System():
    def __init__(self):
        self.init_logger()
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')


        # CONFIG
        config_file = 'config2.json' if os.path.exists('config2.json') else 'config.json'
        with open(config_file) as f:
            data = json.load(f)

        # VISUAL
        self.W = data['resolution']['w']
        self.H = data['resolution']['h']
        self.center_x = self.W/2
        self.center_y = self.H/2

        # GUIDANCE
        self.MIN_THROTTLE = data['guidance']['MIN_THROTTLE']
        self.MAX_THROTTLE = data['guidance']['MAX_THROTTLE']

        self.MIN_SPEED = data['guidance']['MIN_SPEED']
        self.MAX_SPEED = data['guidance']['MAX_SPEED']

        self.MAX_ROLL = data['guidance']['MAX_ROLL']
        self.MAX_PITCH = data['guidance']['MAX_PITCH']
        self.MAX_YAW = data['guidance']['MAX_YAW']
        self.MIN_ALTITUDE = data['guidance']['MIN_ALTITUDE']

        # self.uav_mode = 'GUIDED'
        
        uav_conf = data.get('uav_handler', {})
        port = uav_conf.get('uav_port', '14553')
        self.mavlink_handler = MAVLinkHandler(f'127.0.0.1:{port}')

        # AUTOPILOT 
        # os.system("sudo chmod a+rw /dev/ttyACM0")
        # self.mavlink_handler = MAVLinkHandler('127.0.0.1:14525')

        self.logger.debug('Connected to the aircraft.')
        self.r.set('guid','False')

        # Exit handler
        atexit.register(self.exit_handler)

    def exit_handler(self):
        self.mavlink_handler.master.close()
        self.logger.debug('Connection closed.')

    def update_pos_thread(self):
        while True:
            time.sleep(0.1)
            vehicle_lat, vehicle_lon, alt = self.mavlink_handler.get_location()

    def crash_manuever(self, pitch=30):
        self.logger.debug("Running crash prevention.")
        self.mavlink_handler.set_mode('AUTO')
        self.logger.debug("Mode AUTO.")

        # Manuever
        # while True:
        #     vehicle_lat, vehicle_lon, alt = self.mavlink_handler.get_location()
        #     if alt < self.MIN_ALTITUDE:
        #         self.logger.debug("Altitude is too low, going up.")
        #         self.mavlink_handler.set_target_attitude(0, pitch, thrust=0.7)
        #     else:
        #         break
        #     time.sleep(0.1)


    def init_logger(self):
        # Customcustom logger in order to log to both console and file
        self.logger = logging.Logger('GOAT')
        # Set the log level
        self.logger.setLevel(logging.DEBUG)
        # Create handlers
        c_handler = logging.StreamHandler()
        
        log_file_path = 'GOAT_guidance.log'
        old_logs_dir = "Logs"
        
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            # Generate a unique name for the log file in the old logs directory
            timestamp = datetime.datetime.now()
            new_log_file_name = f"GOAT_guidance_{timestamp}.log"
            new_log_file_path = os.path.join(old_logs_dir, new_log_file_name)
            # Move the log file to the old logs directory
            shutil.move(log_file_path, new_log_file_path)
            
        f_handler = logging.FileHandler(log_file_path)
        # Set levels for handlers
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add it to handlers
        c_format = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(process)d - %(threadName)s - %(filename)s:%(lineno)d - %(funcName)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        # Add handlers to the self.logger
        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)

class GOATGuidance(System):
    def __init__(self):
        super().__init__()
        self.tolerated_error = 5 # in degrees (PID output)
        self.aggressiveness_balance = 2 # will add or subtract from the PID output
        self.sensitivity = 6 # defines number of divisions in error calc

        self.previous_bbox = None
        self.previous_target_time = None

        self.init_control_PID()
        self.logger.debug('PID initialized.')

        self.last_message_time = None  # To track the last message time for mode switching 
        self.speed_valve = False
        self.new_input_start_time = None  # To track when new inputs start after inactivity
        # Start monitoring thread
        monitor_thread = threading.Thread(target=self.monitor_updates, daemon=True)
        monitor_thread.start()
        
        # Mode controller thread in case of inactivity.
        mode_thread = threading.Thread(target=self.check_and_switch_mode, daemon=True)
        mode_thread.start()
        
        threading.Thread(target=self.calculate_speed, daemon=True).start()
        
        # velocity_thread = threading.Thread(target=self.monitor_updates_velocity, daemon=True)
        # velocity_thread.start()

    def calculate_velocity(self, bbox, current_time):
        if self.previous_bbox is None or self.previous_target_time is None:
            self.previous_bbox = bbox
            self.previous_target_time = current_time
            return 0, 0

        time_diff = current_time - self.previous_target_time
        if time_diff == 0:
            return 0, 0

        prev_x1, prev_y1, prev_w, prev_h = self.previous_bbox
        prev_x_center = prev_x1 + (prev_w / 2)
        prev_y_center = prev_y1 + (prev_h / 2)

        x1, y1, w, h = bbox
        x_center = x1 + (w / 2)
        y_center = y1 + (h / 2)

        dx = (x_center - prev_x_center)
        velocity_x =  dx / time_diff

        dy = (y_center - prev_y_center)
        velocity_y = dy / time_diff

        return velocity_x, velocity_y

    # NOT NECESSARY FUNCTION, GOOD FOR SIMULATION
    def check_and_switch_mode(self):
        self.last_message_time = time.time() - 3 
        while True:
            # self.mavlink_handler.set_mode('AUTO')
            self.uav_mode_mav = self.mavlink_handler.get_mode()
            self.r.set('uav_mode', self.uav_mode_mav)
            time.sleep(0.1)
            # self.logger.debug('message time:',self.last_message_time,'uav mode:', self.uav_mode,'last message:', self.last_message_time,'calculation:',(time.time() - self.last_message_time))
            # if self.uav_mode == 'GUIDED' and time.time() - self.last_message_time >= 2:
                # 2 seconds have passed without a new message
                # self.uav_mode = 'AUTO'
                # self.logger.debug("Mode changed to AUTO due to inactivity.")
                # self.logger.debug("inactivity.")



    def monitor_updates(self, timeout=2):
        thread_lock = threading.Lock()
        flag = 0
        while True:
            with thread_lock:
                time_since_last_update = time.time() - self.last_update_time
            if time_since_last_update > timeout:
                with thread_lock:
                    if flag == 0:
                        self.logger.debug('PID has been reset.')
                        self.roll_pid.reset()
                        self.pitch_pid.reset()
                        self.speed_valve = False
                        flag = 1
            else:
                flag = 0
            time.sleep(0.1)

    def monitor_updates_velocity(self, timeout=0.3):
        flag = 0
        while True:
            if self.previous_target_time:
                time_since_last_update = time.time() - self.previous_target_time
                if time_since_last_update > timeout:
                    if flag == 0:
                        self.logger.debug('Target Vx/s has been reset.')
                        # Reset velocity control.
                        self.previous_bbox = None
                        self.previous_target_time = None
                        flag = 1
                            
                else:
                    flag = 0
            time.sleep(0.1)
            
    def minor_correction(self, area,roll=2.5,pitch=2.5,action_time_in_ms=200):
        self.logger.debug("Making minor adjustment.")

        # Convert values according to the area in coord system.
        if area == 1:
            pass
        elif area == 2:
            pitch = -pitch
        elif area == 3:
            roll = -roll
            pitch = -pitch
        elif area == 4:
            roll = -roll

        start_time = time.perf_counter()  # Start the high-resolution timer
        elapsed = 0  # Initialize the elapsed time counter
        while elapsed < action_time_in_ms/1000:  # Continue looping until 200 milliseconds pass
            self.mavlink_handler.set_target_attitude(roll, pitch, thrust=0.7)
            time.sleep(0.01)
            elapsed = time.perf_counter() - start_time  # Update the elapsed time

    def main_controller(self, pid_output_roll, pid_output_pitch, pid_output_yaw):

        def adjust_angle(scaled_output, min_boundary, max_boundary, pieces):
            # Calculate the step size based on the number of pieces
            step_size = (max_boundary - min_boundary) / pieces

            # Generate boundaries and corresponding outputsmax_roll
            boundaries = [min_boundary + i * step_size for i in range(1, pieces)]
            outputs = [min_boundary + i * step_size for i in range(pieces)]

            # Determine the output head on the range
            if abs(scaled_output) < min_boundary:
                return 0
            for i in range(len(boundaries)):
                if abs(scaled_output) <= boundaries[i]:
                    return outputs[i] * (1 if scaled_output >= 0 else -1)

            # For any value exceeding the last boundary, return the maximum scaled by sign
            return max_boundary * (1 if scaled_output >= 0 else -1)

        # Adjust each angle using the same utility function with different parameters
        roll = adjust_angle(pid_output_roll, 4, 35, 5)
        pitch = adjust_angle(pid_output_pitch, 3, 35, 3)
        yaw = adjust_angle(pid_output_yaw, 3, 20, 4)

        return roll, pitch, yaw

    def generalized_normalize(self, data, target_range=(40, 100)):
        normalized_data = stats.gennorm.cdf(data, loc=np.mean(data), scale=np.std(data), beta=2.5)
        normalized_data = np.interp(normalized_data, (0, 1), target_range)
        return normalized_data

    def get_normalized_target(self, center_x, center_y, object_x, object_y, width, height):
        dx = object_x - center_x
        dy = object_y - center_y
        normalized_horizontal = dx/width*2
        normalized_vertical = dy/height*2
        return normalized_horizontal, normalized_vertical

    def calculate_thrust(self):
        # Define coverage limits for min and max throttle
        min_coverage = 3  # in percent
        max_coverage = 7  # in percent

        # Interpolate throttle head on coverage percentage
        if self.horizontal_coverage == 0:
            thrust = 0.65
        elif self.horizontal_coverage <= min_coverage:
            thrust = self.MAX_THROTTLE
        elif self.horizontal_coverage >= max_coverage:
            thrust = self.MIN_THROTTLE
        else:
            thrust = self.MAX_THROTTLE - (self.MAX_THROTTLE - self.MIN_THROTTLE) * (
                    (self.horizontal_coverage - min_coverage) / (max_coverage - min_coverage))

        return thrust

    def calculate_speed(self):
        while self.speed_valve:
            # Define coverage limits for min and max throttle
            # 15 23
            min_coverage = 4  # in percent
            max_coverage = 13  # in percent

            # Interpolate throttle head on coverage percentage
            if self.horizontal_coverage == 0:
                speed = 17
            elif self.horizontal_coverage <= min_coverage:
                speed = self.MAX_SPEED
            elif self.horizontal_coverage >= max_coverage:
                speed = self.MIN_SPEED
            else:
                speed = self.MAX_SPEED - (self.MAX_SPEED - self.MIN_SPEED) * (
                        (self.horizontal_coverage - min_coverage) / (max_coverage - min_coverage))
            speed = speed*100
            self.mavlink_handler.set_parameter_value('TRIM_ARSPD_CM', speed)
            time.sleep(0.2)
    
    def calculate_roll_rate(self, vel_x):
        # Define coverage limits for min and max throttle
        roll_rate = 0
        min_roll_rate = 0.4
        max_roll_rate = 1.5
        
        min_vel = 0  # in percent
        max_vel = 200  # in percent
        
        vel_x = abs(vel_x)

        # Interpolate throttle head on coverage percentage
        if vel_x == 0:
            roll_rate = 0.2
        elif vel_x >= max_vel:
            roll_rate = max_roll_rate
        else:
            roll_rate = max_roll_rate - (max_roll_rate - min_roll_rate) * (
                    (vel_x - min_vel) / (max_vel - min_vel))

        return roll_rate

    def init_control_PID(self):
        self.roll_pid = PID(1, 0.0, 0.1, setpoint=0,
                            output_limits=(-1.0, 1.0))

        self.pitch_pid = PID(1, 0.0, 0, setpoint=-0.1, # setpoint means desired target value, positive setpoint will mean that the enemy will be placed in the lower half of the screen -0.24 corresponds to being fixed higher.
                             output_limits=(-1.0, 1.0))

        self.yaw_pid = PID(1, 0.0, 0.0, setpoint=0,
                            output_limits=(-1.0, 1.0))

        self.last_update_time = time.time()

    def guide_aircraft(self, bbox, current_time): #bbox: x1,y1,w,h
        object_x,object_y = bbox[0] + (bbox[2] / 2), bbox[1] + (bbox[3] / 2)
        obj_x_rel = object_x-self.W/2
        obj_y_rel = -(object_y-self.H/2)
        self.logger.debug('object coords: x: '+str(obj_x_rel)+ ', y: ' + str(obj_y_rel))
        x1,y1,bboxw,bboxh = bbox
        x2 = x1 + bboxw
        y2 = y1 + bboxh
        normalized_horizontal_position, normalized_vertical_position = self.get_normalized_target(self.center_x, self.center_y, object_x, object_y, self.W, self.H)
        
        roll = self.roll_pid(normalized_horizontal_position)
        roll = -roll*self.MAX_ROLL
        
        pitch = self.pitch_pid(normalized_vertical_position)
        pitch = pitch*self.MAX_PITCH
        
        yaw = self.yaw_pid(normalized_horizontal_position)
        yaw = yaw*self.MAX_YAW

            # Check if new inputs have just started after inactivity
        if self.new_input_start_time is None:
            self.new_input_start_time = time.time()

        time_since_new_input = time.time() - self.new_input_start_time


        # Calculates the thrust using the current vertical coverage. (vertical coverage is calculated using yolo output)
        thrust = self.calculate_thrust()
        # speed = (self.calculate_speed())*100
        # self.mavlink_handler.set_parameter_value('TRIM_ARSPD_CM', speed)
        self.logger.debug(f"INPUT: Roll: {roll:.2f}, Pitch: {pitch:.2f}, Thrust: {thrust:.2f}")
        
        # Calculate the bbox velocity
        velocity_x, velocity_y = self.calculate_velocity(bbox, current_time)
        
        dead_area = 7
        if abs(roll) < dead_area:
            self.logger.debug(f"Velocity: x: {velocity_x:.2f}, R<{dead_area} Vx = 0.")
            velocity_x = 0
        else:
            self.logger.debug(f"Velocity: x: {velocity_x:.2f}, Velocity y: {velocity_y:.2f}")
        
        # Adjust roll head on velocity
        max_v_x = 100
        if velocity_x > max_v_x:
            velocity_x = max_v_x
        elif velocity_x < -max_v_x:
            velocity_x = -max_v_x
            
        max_v_y = 50
        if velocity_y > max_v_y:
            velocity_y = max_v_y
        elif velocity_y < -max_v_y:
            velocity_y = -max_v_y            
            
        # Remove velocity in opposite direction
        # if obj_x_rel > 0 and velocity_x < 0:
        #     velocity_x = 0
        # if obj_x_rel < 0 and velocity_x > 0:
        #     velocity_x = 0
            
            
        if self.horizontal_coverage <= 8:
            velocity_factor = 0.05
        else:
            velocity_factor = 0.05
            
        pitch_v_adj = pitch
        # pitch_v_adj += -(velocity_factor * velocity_y)        
            
        roll_v_adj = roll + (velocity_factor * velocity_x)

        self.logger.debug(f"VELOCITY ADJUSTED values: {roll_v_adj:.2f}, Pitch: {pitch_v_adj:.2f}")
        roll = roll_v_adj
        pitch = pitch_v_adj
        
        plus_minus_m_17 = -4
        plus_minus_p_17 = 4

        # if abs(roll)<20:
        #     if roll >0:
        #         roll+=plus_minus_m_17
        #     else:
        #         roll-=plus_minus_m_17
        # else:
        #     if roll>0:
        #         roll+=plus_minus_p_17
        #     else:
        #         roll-=plus_minus_p_17

        if time_since_new_input <= 2:
            multiplier = 0.7  # Apply a multiplier to smooth the roll
            roll *= multiplier
            pitch *= multiplier
            self.logger.debug(f"Applying smooth roll multiplier: {multiplier}. Roll after adjustment: {roll:.2f}")
        
        # ----------------------------------------------------- 
        roll,pitch,yaw = adjust_angles(roll, pitch, yaw, min_roll=3, max_roll=self.MAX_ROLL, min_pitch=2,max_pitch=self.MAX_PITCH, max_yaw=self.MAX_YAW, pieces=30)
        self.logger.debug(f"ANGLE ADJUSTED: Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
        self.logger.debug("-----"*7)
        
        if thrust < 0.55:
            pitch += 3


        # -----------------------------------------------------
        # GUIDANCE
        #self.mavlink_handler.set_target_attitude(roll, pitch, yaw_rate=yaw,use_yaw_rate=True, thrust=thrust)
        self.mavlink_handler.set_target_attitude(roll, pitch, yaw = 0, thrust=thrust, roll_rate=1, throttle_ignore=False)
        # -----------------------------------------------------

    def run(self):
        #self.mavlink_handler.set_mode('GUIDED')
        for message in self.p.listen():
            
            # if self.uav_mode == 'AUTO':
            #     self.uav_mode = 'GUIDED'
            #     self.logger.debug('mode: '+self.uav_mode)
                
            if message['type'] == 'message':
                guid_message = self.r.get('task')
                if guid_message is None:
                    guid_message = b'visual'
                guid_message = guid_message.decode('utf-8').lower()

                if guid_message != 'visual':
                    self.new_input_start_time = None
                    continue

                current_time = time.time()
                if self.last_message_time is None or current_time - self.last_message_time > 2:
                    # Reset the start time for smooth adjustment
                    self.new_input_start_time = None

                self.last_message_time = current_time
                # self.speed_valve = True
                data = message['data']
                data = data.decode('utf-8')

                data = json.loads(data) # data format: x1, y1, w, h, horizontal_coverage
                
                # Check if data is list (from tracker) or dict (from yolo)
                if not isinstance(data, (list, tuple)):
                    # Ignore dictionary messages (likely from detection_final.py)
                    continue
                    
                bbox = (int(data[0]), int(data[1]), int(data[2]), int(data[3]))
                self.horizontal_coverage = data[4]

                self.last_update_time = time.time()
                self.logger.debug(bbox)
                self.guide_aircraft(bbox, self.last_update_time)

                # redis listener is already blocking, no need to sleep

if __name__ == '__main__':
    # will run 2 threads here: attitude controller and minor correction thread
    goat = GOATGuidance()
    goat.run()
