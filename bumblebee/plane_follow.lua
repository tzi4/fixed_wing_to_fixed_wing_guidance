--[[
This file is based on ArduPilot's plane_follow.lua applet and contains local
modifications. It is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See LICENSES/ARDUPILOT-GPL-3.0.txt and
THIRD_PARTY_NOTICES.md for the license and attribution.

Upstream source:
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/plane_follow.lua
SPDX-License-Identifier: GPL-3.0-or-later
--]]

gcs:send_text(0, "In the name of Allah, the Most Gracious, the Most Merciful. When you threw, it was not you who threw, it was Allah who threw it.")
gcs:send_text(0, "SCRIPT HAS STARTED - Plane Follow")
SCRIPT_VERSION = "4.7.0-AZIZ BAŞAR BACK IN BUSINESS"
SCRIPT_NAME = "Plane Follow Org"
SCRIPT_NAME_SHORT = "PFollow"

-- FOLL_ALT_TYPE and Mavlink FRAME use different values 
ALT_FRAME = { GLOBAL = 0, RELATIVE = 1, TERRAIN = 3}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = {GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3,  GLOBAL_TERRAIN_ALT = 10}
MAV_CMD_INT = { ATTITUDE = 30, GLOBAL_POSITION_INT = 33, REQUEST_DATA_STREAM = 66,
                  DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192,
                  CMD_SET_MESSAGE_INTERVAL = 511, CMD_REQUEST_MESSAGE = 512,
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1, DEFAULT = 2} 

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

local ahrs_eas2tas = ahrs:get_EAS2TAS()
local windspeed_vector = ahrs:wind_estimate()

local now = millis():tofloat() * 0.001
local now_target_heading = now
local now_telemetry_request = now
local now_follow_lost = now
local follow_enabled = false
local too_close_follow_up = 0
local save_target_heading1 = -400.0
local save_target_heading2 = -400.0
local save_target_altitude = 0
local tight_turn = false

local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "ZPF_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end
-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 25), 'could not add param table')

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = Parameter('FOLL_ALT_TYPE')
FOLL_SYSID = Parameter('FOLL_SYSID')
FOLL_OFS_Y = Parameter('FOLL_OFS_Y')
local foll_sysid = FOLL_SYSID:get() or -1
local foll_ofs_y = FOLL_OFS_Y:get() or 0.0
local foll_alt_type = FOLL_ALT_TYPE:get() or ALT_FRAME.GLOBAL

-- ZPF Parameters (ORIGINAL)
ZPF_FAIL_MODE = bind_add_param('FAIL_MODE', 1, FLIGHT_MODE.LOITER)
ZPF_EXIT_MODE = bind_add_param('EXIT_MODE', 2, FLIGHT_MODE.LOITER)
ZPF_ACT_FN = bind_add_param("ACT_FN", 3, 301)
ZPF_TIMEOUT = bind_add_param("TIMEOUT", 4, 10)
ZPF_OVRSHT_DEG = bind_add_param("OVRSHT_DEG", 5, 75)
ZPF_TURN_DEG = bind_add_param("TURN_DEG", 6, 15)
ZPF_DIST_CLOSE = bind_add_param("DIST_CLOSE", 7, 50)
ZPF_WIDE_TURNS = bind_add_param("WIDE_TURNS", 8, 1)
ZPF_ALT_OVR = bind_add_param("ALT_OVR", 9, 0)

-- PID Gains
ZPF_D_P = bind_add_param("D_P", 11, 0.02)
ZPF_D_I = bind_add_param("D_I", 12, 0.02)
ZPF_D_D = bind_add_param("D_D", 13, 0.015)

ZPF_V_P = bind_add_param("V_P", 14, 0.02)
ZPF_V_I = bind_add_param("V_I", 15, 0.02)
ZPF_V_D = bind_add_param("V_D", 16, 0.015)

ZPF_LKAHD = bind_add_param("LKAHD", 17, 5)
ZPF_DIST_FUDGE = bind_add_param("DIST_FUDGE", 18, 0.92)
ZPF_SIM_TELF_FN = bind_add_param("SIM_TELF_FN", 19, 302)
ZPF_SR_CH = bind_add_param("SR_CH", 20, -1)
ZPF_SR_INT = bind_add_param("SR_INT", 21, 50)
-- Aziz: Dynamic X parameter to be added to cruise speed (Default: 0 m/s)
ZPF_CRS_ADD = bind_add_param("AZIZ_ADD", 22, 0)

-- Distance based speed limit thresholds
ZPF_SPD_FAR = bind_add_param("SPD_FAR", 23, 100)   -- Far threshold (m)
ZPF_SPD_NEAR = bind_add_param("SPD_NEAR", 24, 30)  -- Near threshold (m)

REFRESH_RATE = 0.05    -- in seconds, so 20Hz
LOST_TARGET_TIMEOUT = (ZPF_TIMEOUT:get() or 10) / REFRESH_RATE
OVERSHOOT_ANGLE = ZPF_OVRSHT_DEG:get() or 75.0
TURNING_ANGLE = ZPF_TURN_DEG:get() or 20.0
DISTANCE_LOOKAHEAD_SECONDS = ZPF_LKAHD:get() or 5.0

local lost_target_countdown = LOST_TARGET_TIMEOUT
local fail_mode = ZPF_FAIL_MODE:get() or FLIGHT_MODE.QRTL
local exit_mode = ZPF_EXIT_MODE:get() or FLIGHT_MODE.LOITER
local use_wide_turns = ZPF_WIDE_TURNS:get() or 1
local distance_fudge = ZPF_DIST_FUDGE:get() or 0.92
local target_serial_channel = ZPF_SR_CH:get() or 0
local simulate_telemetry_failed = false

AIRSPEED_MIN = Parameter('AIRSPEED_MIN')
AIRSPEED_MAX = Parameter('AIRSPEED_MAX')
AIRSPEED_CRUISE = Parameter('AIRSPEED_CRUISE')
WP_LOITER_RAD = Parameter('WP_LOITER_RAD')
WINDSPEED_MAX = Parameter('AHRS_WIND_MAX')

local airspeed_max = AIRSPEED_MAX:get() or 25.0
local airspeed_min = AIRSPEED_MIN:get() or 12.0
local airspeed_cruise = AIRSPEED_CRUISE:get() or 18.0
local windspeed_max = WINDSPEED_MAX:get() or 100.0

local function constrain(v, vmin, vmax)
   if v < vmin then v = vmin end
   if v > vmax then v = vmax end
   return v
end

-- LOADING TO MODULE: modules/speedpid.lua
local status, speedpid = pcall(require, "speedpid")
if not status then
    gcs:send_text(0, "ERROR: 'modules/speedpid.lua' not found!")
    return
end

-- PID INSTANCES (Original SpeedPI Use)
local pid_controller_distance = speedpid.speed_controller(
    ZPF_D_P:get() or 0.01,
    ZPF_D_I:get() or 0.01,
    ZPF_D_D:get() or 0.005,
    0.5, 
    airspeed_min - airspeed_max,  -- Example: 12 - 25 = -13 (Allow deceleration)
    airspeed_max - airspeed_min   -- make boundaries logical (I explain below)
)

local pid_controller_velocity = speedpid.speed_controller(
    ZPF_V_P:get() or 0.01,
    ZPF_V_I:get() or 0.01,
    ZPF_V_D:get() or 0.005,
    2.0, 
    airspeed_min, airspeed_max
)



local mavlink_attitude = require("mavlink_attitude")
local mavlink_attitude_receiver = mavlink_attitude.mavlink_attitude_receiver()

local function follow_frame_to_mavlink(follow_frame)
   local mavlink_frame = MAV_FRAME.GLOBAL;
   if (follow_frame == ALT_FRAME.TERRAIN) then
      mavlink_frame = MAV_FRAME.GLOBAL_TERRAIN_ALT
   end
   if (follow_frame == ALT_FRAME.RELATIVE) then
      mavlink_frame = MAV_FRAME.GLOBAL_RELATIVE_ALT
   end
   return mavlink_frame
end

-- MAVLink Command Helper
local status_cmd, mavlink_command_int = pcall(require, "mavlink_command_int")

local function set_vehicle_target_altitude(target)
   local speed = target.speed or 1000.0 
   if target.alt == nil then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_target_altitude no altitude")
      return
   end
   if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_ALTITUDE, {
                              frame = follow_frame_to_mavlink(target.frame),
                              p3 = speed,
                              z = target.alt }) then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink CHANGE_ALTITUDE returned false")
   end
end

local function set_vehicle_heading(heading)
   local heading_type = heading.type or MAV_HEADING_TYPE.HEADING
   local heading_heading = heading.heading or 0
   local heading_accel = heading.accel or 10.0

   if heading_heading == nil or heading_heading <= -400 or heading_heading > 360 then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_heading no heading")
      return
   end

   if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_HEADING, { frame = MAV_FRAME.GLOBAL,
                                 p1 = heading_type, 
                                 p2 = heading_heading,
                                 p3 = heading_accel }) then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink GUIDED_CHANGE_HEADING failed")
   end
end

local last_speed_log_time = 0 

local function set_vehicle_speed(speed)
   local new_speed = speed.speed or 0.0
   local speed_type = speed.type or MAV_SPEED_TYPE.AIRSPEED
   
   -- CRITICAL FIX: leaving throttle control to autopilot (TECS)
   local throttle = -1.0 
   
   -- Send Command (We Only use DO_CHANGE_SPEED)
   if not gcs:run_command_int(MAV_CMD_INT.DO_CHANGE_SPEED, { 
                              frame = MAV_FRAME.GLOBAL,
                              p1 = speed_type,
                              p2 = new_speed,
                              p3 = throttle }) then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": SPEED COMMAND FAILED!")
   else
      -- SUCCESS MESSAGE (Success is displayed only once per second)
      local now = millis():tofloat() * 0.001
      if (now - last_speed_log_time) > 1.0 then
          gcs:send_text(6, string.format("ZPF_SPEED: The autopilot was ordered %.1f m/s.", new_speed))
          last_speed_log_time = now
      end
   end
end

local function set_vehicle_target_location(target)
   local radius = target.radius or 2.0
   local yaw = target.yaw or 1
   set_vehicle_heading({type = MAV_HEADING_TYPE.DEFAULT})
   if not gcs:run_command_int(MAV_CMD_INT.DO_REPOSITION, { frame = follow_frame_to_mavlink(target.frame),
                              p1 = target.groundspeed or -1,
                              p2 = 1,
                              p3 = radius,
                              p4 = yaw,
                              x = target.lat,
                              y = target.lng,
                              z = target.alt }) then 
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink DO_REPOSITION returned false")
   end
end

local reported_target = true
local lost_target_now = now
local function follow_active()
   local mode = vehicle:get_mode()
   if mode == FLIGHT_MODE.GUIDED then
      if follow_enabled then
        if follow:have_target() then
            reported_target = true
            lost_target_now = now
         else
            if reported_target then
               if (now - lost_target_now) > 5 then
                  gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": lost prior target: " .. tostring(follow:get_target_sysid()))
                  lost_target_now = now
               end
            end
            reported_target = false
        end
      end
   else
      reported_target = false
   end
   return reported_target
end

local last_follow_active_state = rc:get_aux_cached(ZPF_ACT_FN:get())
local last_tel_fail_state = rc:get_aux_cached(ZPF_SIM_TELF_FN:get())

local function follow_check()
    -- AUTOMATIC START
    if vehicle:get_mode() == FLIGHT_MODE.GUIDED and not follow_enabled and arming:is_armed() then
        gcs:send_text(6, "ZPF: Auto-Start (GUIDED)")
        follow_enabled = true
        lost_target_countdown = LOST_TARGET_TIMEOUT
        -- FIX: This is important, when the speedpi module is used, reset is called like this:
        pid_controller_distance.reset()
        pid_controller_velocity.reset()
    end

   if ZPF_ACT_FN == nil then return end
   local foll_act_fn = ZPF_ACT_FN:get()
   if foll_act_fn == nil then return end
   local active_state = rc:get_aux_cached(foll_act_fn)
   if (active_state ~= last_follow_active_state) then
      if( active_state == 0) then
         if follow_enabled then
            vehicle:set_mode(exit_mode)
            follow_enabled = false
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": disabled")
         end
      elseif (active_state == 2) then
         if not (arming:is_armed()) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": must be armed")
         end
         vehicle:set_mode(FLIGHT_MODE.GUIDED)
         follow_enabled = true
         lost_target_countdown = LOST_TARGET_TIMEOUT
         
         pid_controller_distance.reset()
         pid_controller_velocity.reset()
         gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": enabled")
      end
      last_follow_active_state = active_state
   end
   local sim_tel_fail = ZPF_SIM_TELF_FN:get()
   local tel_fail_state = rc:get_aux_cached(sim_tel_fail)
   if tel_fail_state ~= last_tel_fail_state then
      if tel_fail_state == 0 then
         simulate_telemetry_failed = false
      else
         simulate_telemetry_failed = true
      end
      last_tel_fail_state = tel_fail_state
   end
end

local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then res = res + 360.0 end
    return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then res = res - 360 end
    return res
end

local function calculate_airspeed_from_groundspeed(velocity_vector)
   local airspeed_vector = velocity_vector - windspeed_vector
   local airspeed = airspeed_vector:length()
   airspeed = airspeed * ahrs_eas2tas
   airspeed = constrain(airspeed, airspeed - windspeed_max, airspeed + windspeed_max)
   airspeed = airspeed / ahrs_eas2tas
   return airspeed
end

-- main update function

local last_command_send_time = 0
local debug_target_alt = 0
local debug_desired_hdg = 0

-- MAIN UPDATE FUNCTION (STABLE TAIL-FOLLOWING MODE)
local function update()
   now = millis():tofloat() * 0.001
   ahrs_eas2tas = ahrs:get_EAS2TAS()
   windspeed_vector = ahrs:wind_estimate()
   local current_cruise = AIRSPEED_CRUISE:get() or 18.0
   local crs_add = ZPF_CRS_ADD:get() or 5.0
   local absolute_max = AIRSPEED_MAX:get() or 25.0
   
   -- Our new upper limit formula:
   local dynamic_max_speed = current_cruise + crs_add
   
   -- SECURITY LOCK: If you accidentally enter the X value too high,
   -- prevents the aircraft from exceeding the physical AIRSPEED_MAX limit.
   dynamic_max_speed = math.min(dynamic_max_speed, absolute_max)

   follow_check()
   if not follow_active() then return end

   local altitude_override = ZPF_ALT_OVR:get() or 0

   -- TARGET SEND (Parameter FOLL_OFS_X = -40 is processed here)
   local target_location, target_velocity = follow:get_target_location_and_velocity()
   local target_location_offset, target_velocity_offset = follow:get_target_location_and_velocity_ofs()
   

   local target_heading = follow:get_target_heading_deg() or -400
      local current_location = ahrs:get_location()

      if current_location == nil then return end

      -- Measure distance (Distance to virtual point)
      local xy_dist = 0
      if target_location_offset and current_location then
         xy_dist = current_location:get_distance(target_location_offset)
      end

      local current_altitude = current_location:alt() * 0.01
      local vehicle_airspeed = ahrs:airspeed_estimate()
      

   -- Data Security
   if target_location == nil or target_location_offset == nil or
      target_velocity == nil or target_velocity_offset == nil or
      target_heading <= -400 then
      
      lost_target_countdown = lost_target_countdown - 1
      if lost_target_countdown <= 0 then
         follow_enabled = false
         vehicle:set_mode(fail_mode)
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": TARGET LOST")
         return
      end
      return
   else
      lost_target_countdown = LOST_TARGET_TIMEOUT
      now_follow_lost = now
   end

   local target_airspeed = calculate_airspeed_from_groundspeed(target_velocity_offset)
   
   -- === DIRECTION CALCULATION (SIMPLE AND CLEAR) ===
   -- I DELETED the complex parallel flight logic.
   -- The plane looks only at the point it needs to go (Offset point).
   local heading_to_offset = math.deg(current_location:get_bearing(target_location_offset))






   local desired_heading = heading_to_offset
   if xy_dist < 30 and target_heading > -400 then
       local blend = xy_dist / 30.0
       local diff = wrap_180(heading_to_offset - target_heading)
       desired_heading = wrap_360(target_heading + diff * blend)
   end



   -- === COMMAND TIMER ===
   local target_altitude = 0.0
   if (now - last_command_send_time) > 0.2 then
       
       if altitude_override ~= 0 then
          target_altitude = altitude_override
       elseif target_location_offset ~= nil then
          target_location_offset:change_alt_frame(foll_alt_type)
          target_altitude = target_location_offset:alt() * 0.01
       end
    
       set_vehicle_heading({heading = desired_heading})
       gcs:send_named_float("CMD_HDG", desired_heading)  -- Heading to the interface







       -- altitude low-pass filter (prevents oscillation)
       if save_target_altitude == 0 then
           save_target_altitude = target_altitude
       end
       save_target_altitude = save_target_altitude + 0.50 * (target_altitude - save_target_altitude)
       set_vehicle_target_altitude({alt = save_target_altitude, frame = foll_alt_type})
       




       
       debug_target_alt = target_altitude
       debug_desired_hdg = desired_heading
       
       last_command_send_time = now
   end





   -- ═══ DISTANCE BASED SPEED LIMIT ═══
   -- Adds coarse speed control on top of PID
   -- Far: max speed, near: target speed, between: gradual
   -- NOTE: Parameters are read in each cycle and change instantly during flight.
   local spd_far = ZPF_SPD_FAR:get() or 100.0
   local spd_near = ZPF_SPD_NEAR:get() or 30.0
   -- Safety: far must always be greater than near (prevents division by zero)
   if spd_far <= spd_near then
       spd_far = spd_near + 1.0
   end

   local speed_bias = 0.0
   if xy_dist > spd_far then
       speed_bias = dynamic_max_speed - target_airspeed -- THIS HAS CHANGED
   elseif xy_dist > spd_near then
       local ratio = (xy_dist - spd_near) / (spd_far - spd_near)
       speed_bias = ratio * (dynamic_max_speed - target_airspeed) -- THIS HAS CHANGED
   else
       speed_bias = 0.0
   end
   -- Maintain at least a +3 m/s speed margin while far from the target
   if xy_dist > spd_near then
       speed_bias = math.max(speed_bias, 3.0)
   end


   -- SPEED CONTROLLER (PID)
   local dist_error = xy_dist 
   
   -- Basic Overshoot Control (For Braking Only)
   -- If the point I need to go to is behind me (>90 degrees difference), we give a negative value to the PID to brake.
   local angle_diff = math.abs(wrap_180(heading_to_offset - target_heading))
   if angle_diff > 90 then
       dist_error = -dist_error
   end

   -- local dv_error = dist_error * REFRESH_RATE
   local dv_error = dist_error * 0.2
   local airspeed_new = vehicle_airspeed
   local dv = 0.0

   if dv_error ~= nil then
      if math.abs(dv_error) < 0.0001 then
         if dv_error >= 0 then dv_error = 0.0001 else dv_error = -0.0001 end
      end

      dv = pid_controller_distance.update(target_airspeed - vehicle_airspeed, dv_error)
      airspeed_new = pid_controller_velocity.update(vehicle_airspeed, dv)





      -- Speed ​​ceiling according to distance
      local speed_ceiling = target_airspeed + speed_bias
      if airspeed_new > speed_ceiling then
          airspeed_new = speed_ceiling
      end





      airspeed_new = constrain(airspeed_new, airspeed_min, dynamic_max_speed)
      set_vehicle_speed({speed = airspeed_new})

      if (millis():tofloat()*0.001 % 1) < 0.1 then
         -- gcs:send_text(6, string.format("TAIL: Dist:%.1f Spd:%.1f Hdg:%.1f", xy_dist, airspeed_new, desired_heading))
         -- gcs:send_text(6, string.format("TAIL: D:%.0f S:%.1f H:%.0f A:%.0f/%.0f",
         --    xy_dist, airspeed_new, desired_heading, current_altitude, target_altitude or 0))


         gcs:send_text(6, string.format("TAIL: D:%.0f Speed:%.1f/%.1f Alt:%.0f/%.0f(%+.0f) Bias:%.1f",
            xy_dist, vehicle_airspeed, target_airspeed, current_altitude, save_target_altitude, save_target_altitude - current_altitude, speed_bias))
      end
   end
end

local function protected_wrapper()
   local success, err = pcall(update)

   if not success then
      gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. "Internal Error: " .. err)
      return protected_wrapper, 1000
   end
   return protected_wrapper, 1000 * REFRESH_RATE
end

local function delayed_startup()
   gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
   return protected_wrapper()
end

if FWVersion:type() == 3 then
   if arming:is_armed() then
      return delayed_startup, 1000
   else
      return delayed_startup, 20000
   end
else
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
