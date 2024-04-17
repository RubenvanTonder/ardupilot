-- Sail starting position is 30 deg, mark with a permanent marker the rope
local sail_angle = 2
-- sail angle 30 => 0
-- sail angle 45 => 1 
-- sail angle 60 => 2

-- Roll offset
local offset = -7/180*math.pi
local bat_volt = battery:voltage(0)
gcs:send_text(6,"Battery Voltage " .. bat_volt)
local nominal_voltage = 7.88
local diff = (bat_volt - nominal_voltage)/0.2
local rest_pwm = 1780 + diff*100
local winch_out_pwm = rest_pwm + 90 + diff*100
local winch_in_pwm = rest_pwm - 90 + diff*100
local sail_servo = 95
local servo_function = Parameter()
servo_function:init("SERVO3_FUNCTION")
servo_function:set(0)

-- State 2 where the servo is stopped
function stop()
    gcs:send_text(6,"Stop")
    SRV_Channels:set_output_pwm(sail_servo,math.floor(tonumber(rest_pwm)))
    return update,500
end

-- State 1 where the servo is winching in the sail
function winch_in()
    gcs:send_text(6,"winch_in")
    SRV_Channels:set_output_pwm(sail_servo,math.floor(tonumber(winch_in_pwm)))
    return stop, 225
end

-- State 0 where the servo is winching out the sail
function winch_out()
    gcs:send_text(6,"winch_out")
    SRV_Channels:set_output_pwm(sail_servo,math.floor(tonumber(winch_out_pwm)))
    return stop, 225
end

local AUX_FUNCTION_NUM = 301
local servo_on = false
local scripting_rc_1 = rc:find_channel_for_option(AUX_FUNCTION_NUM)
local function update()
    
    --Log Sail Angle
    logger.write("SAI",'SailPos','f',sail_angle)
    if scripting_rc_1:norm_input() > 0 then
        -- Turn on Sail servo
        if servo_on then
            servo_function:set(sail_servo)
            servo_on = not(servo_on)
        end

        -- State 0: Check if roll angle is greater than ideal
        if (math.deg(math.abs(ahrs:get_roll()+offset))) > 10 then  
            -- Check if sail can be winched out further
            if sail_angle < 2 then 
                sail_angle = sail_angle+1
                gcs:send_text(6,"Sail Angle " .. sail_angle)
                winch_out()
            else 
                stop()
            end
        
        -- State 1: Check if roll angle is less than ideal
        elseif (math.deg(math.abs(ahrs:get_roll()+offset))) < 5  then 
            -- Check if sail can be winched in further
            if sail_angle > 0 then 
                sail_angle = sail_angle -1
                gcs:send_text(6,"Sail Angle " .. sail_angle)
                winch_in()
            else
                stop()
            end

        -- State 2: Roll angle in ideal range do nothing
        else 
            -- do nothing
            --stop()
            return update, 500
        end
    else

        if not(servo_on) then
            servo_function:set(0)
            servo_on = not(servo_on)
        end
    end
    return update, 500
end

return update()