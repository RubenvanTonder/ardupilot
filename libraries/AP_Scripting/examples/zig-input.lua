-- This scirpt uses Rover's turn rate controller to make the vehicle follow a zigzag step input

-- Edit these variables
local desired_heading_rad = math.pi/8     -- Desired step heaiding in radians
local target_speed = 0.5     -- target speed in m/s(Does not matter in for Sailboat)

-- Fixed variables
local desired_heading_deg = 10
local reference_heading =0;
local rover_guided_mode_num = 15
local target_yaw_rate = 0
local current_yaw = 0
local servo_function = Parameter()
local servo_func = servo_function:init("SERVO1_FUNCTION")
local ground_steer =26
local rc_in = 51

-- Script Start --

gcs:send_text(0,"Script started")
gcs:send_text(0,"Step Input: " .. tostring(desired_heading_deg))


function update()
    --gcs:send_text(0,"Update")
   if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num then
     --param:set("SERVO9_FUNCTION", ground_steer)
     reference_heading = ahrs:get_yaw() * 180 / math.pi
     gcs:send_text(6,"Step Started: "..reference_heading)
    return yaw_rate_controller, 500
   end
    return update, 100
end

function yaw_rate_controller()
    current_yaw = ahrs:get_yaw() * 180 / math.pi
    gcs:send_text(6, "Current Yaw: "..target_yaw_rate)
    target_yaw_rate = ((reference_heading + desired_heading_deg) - current_yaw)
    gcs:send_text(6, "Zig: "..target_yaw_rate)

    -- send guided message
    if not vehicle:set_desired_turn_rate_and_speed(-10,target_speed) then
        gcs:send_text(0, "Failed to send target ")
    end
    if not arming:is_armed() then
        --param:set("SERVO9_FUNCTION", ground_steer)
        return update,100
    end

    return yaw_rate_controller,50
end

return update()
