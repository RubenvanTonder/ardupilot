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


local servoTwo_function = Parameter()
servoTwo_function:init('SERVO9_FUNCTION')
local servoOriginal = servoTwo_function:get()

local parameterTwo = servoTwo_function:get()
gcs:send_text(6, "SERVO9_FUNCTION= "..parameterTwo)

function update()
        if arming:is_armed() then
            if  vehicle:get_mode()==0  then
            output_pwm = SRV_Channels:get_output_pwm(K_rudder)
            gcs:send_text(6, "Step Output "..1500+step_8)
            SRV_Channels:set_output_pwm(K_rudder, 1500+_step_2)              
            return update, 50
            else
            servoTwo_function:set(servoOriginal)
            return start, 50
            end
        else
            servoTwo_function:set(servoOriginal)
            return start, 10
        end
    return update, 2000
end

function start()
    if arming:is_armed() then
        if vehicle:get_mode()==0  then
            yaw_angle = ahrs:get_yaw();
            if yaw_angle > math.pi then
                yaw_angle = yaw_angle - 2*math.pi
            end
            gcs:send_text(6, "Yaw Angle: "..yaw_angle)
            servoTwo_function:set(K_rudder);
            return update, 50
        end
        return start,50
    end
    return start, 50
end

return start, 100
