-- Testing set_output_pwm_chan_timeout and get_output_pwm
--
-- This will set MAIN1 servo to 1700 pwm for 1 second,
-- then assigned function behavior for 1 second, and then 1100 pwm for 1 second

local zigzag = true
local K_rudder = 94
local rudder_channel = SRV_Channels:find_channel(K_rudder)
local yaw_angle = 0;
local current_yaw_angle =0;
local servoValue = 94

--Ratio of servo output to rudder angle 0.0666 (deg/per servo step)
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


local servoTwo_function = Parameter()
servoTwo_function:init('SERVO1_FUNCTION')
local servoOriginal = servoTwo_function:get()

local parameterTwo = servoTwo_function:get()
gcs:send_text(6, "SERVO1_FUNCTION= "..parameterTwo)

local aux_changed = false
local AUX_FUNCTION_NUM = 300
local MAV_SEVERITY_INFO = 6

function update()
        if arming:is_armed() then
            if  vehicle:get_mode()==0  then
            gcs:send_text(6, "Step Output "..1500+step_6)
            SRV_Channels:set_output_pwm(K_rudder, 1500+step_6)              
            return update, 50
            else
            servoTwo_function:set(servoOriginal)
            return start, 50
            end
        else
            servoTwo_function:set(servoOriginal)
            return start, 50
        end
end

function start()
    if arming:is_armed() then
        local aux_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
        if aux_pos ~= LAST_AUX_POS then
            LAST_AUX_POS = aux_pos
            aux_changed = not(aux_changed)
            gcs:send_text(MAV_SEVERITY_INFO, string.format("Aux set to %u", aux_pos))
        end
        -- Initialize Rudder Step
        if aux_changed then
            gcs:send_text(MAV_SEVERITY_INFO,"Step Run")
            servoTwo_function:set(K_rudder);
            return update, 100
        end
        return start, 100
    end
    return start, 100
end

return start, 100
