-- Sail starting position is 30 deg, mark with a permanent marker the rope
local sail_angle = 0
-- sail angle 30 => 0
-- sail angle 45 => 1 
-- sail angle 60 => 2

-- Roll offset
local offset = -4

-- State 2 where the servo is stopped
function stop()
    gcs:send_text(6,"Stop")
    SRV_Channels:set_output_pwm(95,1600)
    return update()
end

-- State 1 where the servo is winching in the sail
function winch_in()
    gcs:send_text(6,"winch_in")
    SRV_Channels:set_output_pwm(95,1700)
    return update()
end

-- State 0 where the servo is winching out the sail
function winch_out()
    gcs:send_text(6,"winch_out")
    SRV_Channels:set_output_pwm(95,1500)
    return update()
end

local function update()

    -- State 0: Check if roll angle is greater than ideal
    if (ahrs:get_roll()+offset) > 10 then 
        
        -- Check if sail can be winched out further
        if sail_angle < 2 then 
            winch_out()
        elseif
            stop()
        end
    
    -- State 1: Check if roll angle is less than ideal
    elseif (ahrs:get_roll()+offset) < 5  then 

        -- Check if sail can be winched in further
        if sail_angle > 0 then 
            winch_out()
        elseif
            stop()
        end
    -- State 2: Roll angle in ideal range do nothing
    else 
        -- do nothing
        stop()
        return update, 500
    end
    return update, 500
end

return update