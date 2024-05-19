--[[
    Script to control the rate at which the sail is winched in and out, differs slighty from real implementation that is why there is two scripts
--]]

local sail_angle = 2
-- sail angle 30 => 0
-- sail angle 45 => 1 
-- sail angle 60 => 2

-- Roll offset
local offset = 0
local sail_60 = 1800 
local sail_45 = 1700
local sail_30 = 1600
local sail_servo = 95
local servo_function = Parameter()
servo_function:init("SERVO3_FUNCTION")
servo_function:set(0)

local AUX_FUNCTION_NUM = 301
local servo_on = false
local function update()
    
    --Log Sail Angle
    logger.write("SAI",'SailPos','f',sail_angle)
    if rc:get_aux_cached(AUX_FUNCTION_NUM) == 2 then
        -- Turn on Sail servo
        servo_function:set(sail_servo)


        -- State 0: Check if roll angle is greater than ideal
        if (math.deg(math.abs(ahrs:get_roll()+offset))) > 10 then 

            -- Check if sail can be winched out further
            if sail_angle < 2 then 
                sail_angle = sail_angle+1
                gcs:send_text(6,"Sail Angle " .. sail_angle)
                if sail_angle == 0 then
                    SRV_Channels:set_output_pwm(sail_servo, math.floor(tonumber(sail_45)))
                    sail_angle = sail_angle +1
                elseif sail_angle == 1 then
                    SRV_Channels:set_output_pwm(sail_servo, math.floor(tonumber(sail_60)))
                    sail_angle = sail_angle +1
                end
              
            end
        
        -- State 1: Check if roll angle is less than ideal
        elseif (math.deg(math.abs(ahrs:get_roll()+offset))) < 5  then 
            -- Check if sail can be winched in further
            if sail_angle > 0 then 
                sail_angle = sail_angle -1
                gcs:send_text(6,"Sail Angle " .. sail_angle)
                if sail_angle == 2 then
                    SRV_Channels:set_output_pwm(sail_servo, math.floor(tonumber(sail_45)))
                    sail_angle = sail_angle - 1
                elseif sail_angle == 1 then
                    SRV_Channels:set_output_pwm(sail_servo, math.floor(tonumber(sail_60)))
                    sail_angle = sail_angle - 1
                end
            end

        -- State 2: Roll angle in ideal range do nothing
        else 
            -- do nothing
            --stop()
            return update, 500
        end
    else
        servo_function:set(0)
    end
    return update, 1000
end

return update()