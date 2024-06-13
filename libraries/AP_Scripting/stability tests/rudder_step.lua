--[[
    Script to perform rudder step responses
--]]
local K_rudder = 94
local servo_Original = 26
local servo_function = Parameter()
servo_function:init('SERVO1_FUNCTION')

--Ratio of serco output to rudder angle 0.0666 (deg/per servo step)
-- 10 deg rudder angle = 150
local step_2 = 30
local step_4 = 60
local step_6= 80
local step_8 = 120
local step_10 = 150

local _step_2 = -30
local _step_4 = -60
local _step_6= -80
local _step_8 = -120
local _step_10 = -150

local step_enable = 0

local step_angle = step_8
local AUX_FUNCTION_NUM = 300
gcs:send_text(6,"Step: run " ..math.floor(step_angle/150*10))
-- Update
function update()

      -- Initialize the rudder step
        if rc:get_aux_cached(AUX_FUNCTION_NUM) == 2 then
            step_enable = 1
            -- Initialize PI Controller
            servo_function:set(K_rudder);
        else
            step_enable = 0
            servo_function:set(servo_Original)
            SRV_Channels:set_output_pwm(94,1500)
        end

        if arming:is_armed() and step_enable == 1 then
            -- Set servo output to specific angle
            SRV_Channels:set_output_pwm(K_rudder,1500+step_angle)
        end

    return update, 50
end

return update, 100
